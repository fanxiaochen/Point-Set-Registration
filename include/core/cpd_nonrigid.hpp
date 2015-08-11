/**
 * @file
 * @author  Xiaochen Fan <fan.daybreak@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * 
 */

#ifndef CPD_NRIGID_HPP
#define CPD_NRIGID_HPP

#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>

#include <Eigen/SVD>

#include "core/cpd_base.hpp"
#include "base/parameters.hpp"
#include "fast/fgt_wrapper.hpp"
#include "fast/low_rank.hpp"
#include "disp/render_thread.hpp"

namespace cpd
{
    template <typename T, int D>
    class CPDNRigid: public CPDBase<T, D>
    {
    public:
        CPDNRigid();
        virtual ~CPDNRigid();

        void setLambda(T lambda);
        void setBeta(T beta);
        inline NRigidParas<T, D>& getParameters(){ return _paras; }

        inline TMatrix& getG(){ return _G; }

        void run();

    private:
        void initialization();
        void e_step();
        void m_step();
        void align();
        void correspondences();

        void constructG();
        T computeGaussianExp(size_t m, size_t n);
        T energy();

        void P1_correlation();

    private:
        NRigidParas<T, D>   _paras;
        TMatrix             _G;
        TMatrix             _Q;
        TMatrix             _S;
    };
}

namespace cpd
{
    template <typename T, int D>
    CPDNRigid<T, D>::CPDNRigid()
    {
        this->_type = NONRIGID;

        _paras._lambda = 2;
        _paras._beta = 2;
    }

    template <typename T, int D>
    CPDNRigid<T, D>::~CPDNRigid(){}

    template <typename T, int D>
    void CPDNRigid<T, D>::setLambda(T lambda)
    {
        _paras._lambda = lambda;
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::setBeta(T beta)
    {
        _paras._beta = beta;
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::run()
    {
        size_t iter_num = 0;
        T e_tol = 10 + this->_e_tol;
        T e = 0;
        
        this->normalize();
        initialization();

        if (this->_vision)
        {
            RenderThread<T, D>::instance()->updateModel(this->_model);
            RenderThread<T, D>::instance()->updateData(this->_data);
            RenderThread<T, D>::instance()->startThread();
        }

        while (iter_num < this->_iter_num && e_tol > this->_e_tol && _paras._sigma2 > 10 * this->_v_tol)
        {

            e_step();
            
            T old_e = e;
            e = energy();
            e += _paras._lambda/2 * (_paras._W.transpose()*_G*_paras._W).trace();
            e_tol = fabs((e - old_e) / e);

            m_step();

            if (this->_vision == true)
                RenderThread<T, D>::instance()->updateModel(this->_T);

            iter_num ++;
        }
        
        correspondences();
        this->updateModel();
        this->denormalize();
        RenderThread<T, D>::instance()->cancel();
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::correspondences()
    {
        this->_corres.setZero(this->_M, this->_N);

        for (size_t n = 0; n < this->_N; n ++)
        {
            typename std::vector<T> t_exp;
            T sum_exp = 0;
            T c = pow((2*M_PI*_paras._sigma2), 0.5*D) * (this->_w/(1-this->_w)) * (T(this->_M)/this->_N);
            for (size_t m = 0; m < this->_M; m ++)
            {
                T m_exp = computeGaussianExp(m, n);
                t_exp.push_back(m_exp);
                sum_exp += m_exp;
            }

            for (size_t m = 0; m < this->_M; m ++)
            {
                this->_corres(m, n) = t_exp.at(m) / (sum_exp + c);
            }
        }
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::initialization()
    {

        _paras._W = MatrixType<T, D>::Matrix::Zero(this->_M, D);

        T sigma_sum = this->_M*(this->_data.transpose()*this->_data).trace() + 
            this->_N*(this->_model.transpose()*this->_model).trace() - 
            2*(this->_data.colwise().sum())*(this->_model.colwise().sum()).transpose();
        _paras._sigma2 = sigma_sum / (D*this->_N*this->_M);

        this->initTransform();
        constructG();
        
        if (this->_lr)
            lr_approximate<T, D>(_G, _Q, _S, this->_K, this->_lr_maxitr);
    }

    template<typename T, int D>
    void CPDNRigid<T, D>::e_step()
    {
        if (!this->_fgt)
        {
            correspondences();
            this->_P1 = this->_corres * TVector(this->_N).setOnes();
            this->_PT1 = this->_corres.transpose() * TVector(this->_M).setOnes();
            this->_PX = this->_corres * this->_data;
        }
        else
        {
            T c = pow((2*M_PI*_paras._sigma2), 0.5*D) * (this->_w/(1-this->_w)) * (T(this->_M)/this->_N);
            TMatrix KT1 = fgt<T, D>(this->_T, this->_data, TVector(this->_M).setOnes(), sqrt(2*_paras._sigma2), this->_fgt_eps);
            TVector a = (TVector(KT1) + c*TVector(this->_N).setOnes()).cwiseInverse();

            TMatrix aX = MatrixType<T, D>::Matrix::Zero(this->_N, D);
            for (size_t i = 0; i < D; i ++)
            {
                aX.col(i) = this->_data.col(i).cwiseProduct(a);
            }

            this->_PT1 = TVector(this->_N).setOnes() - c * a;
            this->_P1 = fgt<T, D>(this->_data, this->_T, a, sqrt(2*_paras._sigma2), this->_fgt_eps);
            this->_PX = fgt<T, D>(this->_data, this->_T, aX, sqrt(2*_paras._sigma2), this->_fgt_eps);
        }
    }

    template<typename T, int D>
    void CPDNRigid<T, D>::m_step()
    {
        T N_P = this->_P1.sum();

        if (!this->_lr)
        {
            TMatrix A = (this->_P1.asDiagonal()*_G + _paras._lambda*_paras._sigma2*MatrixType<T, D>::Matrix::Identity(this->_M, this->_M));
            TMatrix B = this->_PX - this->_P1.asDiagonal() * this->_model;
            _paras._W = A.inverse() * B;
        }
        else
        {
            P1_correlation();
            TMatrix A1 = ((1/(_paras._lambda*_paras._sigma2))*this->_P1).asDiagonal();
            TMatrix A2 = _Q * (_S.inverse() + _Q.transpose()*A1*_Q).inverse() * _Q.transpose();
            TMatrix A_inv = A1 - A1 * A2 * A1;
            TMatrix B = this->_P1.cwiseInverse().asDiagonal() * this->_PX - this->_model;
            _paras._W = A_inv * B;
        }

        align();

        _paras._sigma2 = 1/(N_P*D) * ((this->_data.transpose()*this->_PT1.asDiagonal()*this->_data).trace() -
            2*(this->_PX.transpose()*this->_T).trace() + (this->_T.transpose()*this->_P1.asDiagonal()*this->_T).trace());
        _paras._sigma2 = fabs(_paras._sigma2);

    }

    template<typename T, int D>
    void CPDNRigid<T, D>::align()
    {
        this->_T = this->_model + _G * _paras._W;
    }

    template <typename T, int D>
    T CPDNRigid<T, D>::energy()
    {
        T e = 0;
        
        for (size_t n = 0; n < this->_N; n ++)
        {
            T sp = 0;
            for (size_t m = 0; m < this->_M; m ++)
            {
                sp += computeGaussianExp(m, n);
            }

            sp += pow((2*M_PI*_paras._sigma2), 0.5*D) * (this->_w/(1-this->_w)) * (T(this->_M)/this->_N);

            e += -log(sp);

        }

        e += this->_N * D * log(_paras._sigma2) / 2;

        return e;
    }

    template <typename T, int D>
    T CPDNRigid<T, D>::computeGaussianExp(size_t m, size_t n)
    {
        TVector vec = TVector(this->_T.row(m) - this->_data.row(n));
        T g_exp = exp(-vec.squaredNorm()/(2*_paras._sigma2));
        return g_exp;
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::constructG()
    {
        _G = MatrixType<T, D>::Matrix::Zero(this->_M, this->_M);

        for (size_t i = 0; i < this->_M; i ++)
        {
            for (size_t j = 0; j < this->_M; j ++)
            {
                _G(i, j) = exp(-TVector(this->_model.row(i)-this->_model.row(j)).squaredNorm()/(2*_paras._beta*_paras._beta));
            }
        }
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::P1_correlation()
    {
        T min_numerics = std::numeric_limits<T>::epsilon();

        for (size_t i = 0, i_end = this->_P1.rows(); i < i_end; i ++)
        {
            if (this->_P1(i, 0) < min_numerics)
                this->_P1(i, 0) = min_numerics;
        }
    }
}



#endif
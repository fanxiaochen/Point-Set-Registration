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

#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>

#include <Eigen/SVD>

#include "core/cpd_base.hpp"
#include "base/parameters.hpp"
#include "fast/fgt_wrapper.hpp"
#include "disp/render_thread.hpp"

namespace cpd
{
    template <typename T, int D>
    class CPDRigid: public CPDBase<T, D>
    {
    public:
        CPDRigid();
        virtual ~CPDRigid();

        inline RigidParas<T, D>& getParameters(){ return _paras; }

        void run();

    private:
        void initialization();
        void e_step();
        void m_step();
        void align();
        void correspondences();

        T computeGaussianExp(size_t m, size_t n);
        T energy();

    private:
        RigidParas<T, D>    _paras;
    };
}

namespace cpd
{
    template <typename T, int D>
    CPDRigid<T, D>::CPDRigid()
    {
        this->_type = RIGID;
    }

    template <typename T, int D>
    CPDRigid<T, D>::~CPDRigid(){}

    template <typename T, int D>
    void CPDRigid<T, D>::run()
    {
        size_t iter_num = 0;
        T e_tol = 10 + this->_e_tol;
        T e = 0;
        
        normalize();
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
            e_tol = abs((e - old_e) / e);

            m_step();
            align();

            if (this->_vision == true)
                RenderThread<T, D>::instance()->updateModel(this->_T);

            iter_num ++;	
        }
        
        correspondences();
        updateModel();
        denormalize();
        RenderThread<T, D>::instance()->cancel();		
    }

    template <typename T, int D>
    void CPDRigid<T, D>::initialization()
    {
        // determine data number
        this->_M = this->_model.rows();
        this->_N = this->_data.rows();

        // initialization
        _paras._R = TMatrix::Identity(D, D);
        _paras._t = TVector::Zero(D, 1);
        _paras._s = 1;

        T sigma_sum = this->_M*(this->_data.transpose()*this->_data).trace() + 
            this->_N*(this->_model.transpose()*this->_model).trace() - 
            2*(this->_data.colwise().sum())*(this->_model.colwise().sum()).transpose();
        _paras._sigma2 = sigma_sum / (D*this->_N*this->_M);

        initTransform();
    }

    

    template <typename T, int D>
    void CPDRigid<T, D>::correspondences()
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
    void CPDRigid<T, D>::e_step()
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

            TMatrix aX = TMatrix::Zero(this->_N, D);
            for (size_t i = 0; i < D; i ++)
            {
                aX.col(i) = this->_data.col(i).cwiseProduct(a);
            }

            this->_PT1 = TVector(this->_N).setOnes() - c * a;
            this->_P1 = fgt<T, D>(this->_data, this->_T, a, sqrt(2*_paras._sigma2), this->_fgt_eps);
            this->_PX = fgt<T, D>(this->_data, this->_T, aX, sqrt(2*_paras._sigma2), this->_fgt_eps);
        }
    }

    template <typename T, int D>
    void CPDRigid<T, D>::m_step()
    {
        T N_P = this->_P1.sum();
        TVector mu_x = this->_data.transpose() * this->_PT1 / N_P;
        TVector mu_y = this->_model.transpose() * this->_P1 / N_P;

        TMatrixD X_hat = this->_data - TMatrix(TVector(this->_N).setOnes() * mu_x.transpose());
        TMatrixD Y_hat = this->_model - TMatrix(TVector(this->_M).setOnes() * mu_y.transpose());

        TMatrix A = (this->_PX-this->_P1*mu_x.transpose()).transpose() * 
            (this->_model - TMatrix(TVector(this->_M).setOnes() * mu_y.transpose()));

        Eigen::JacobiSVD<TMatrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        TMatrix U = svd.matrixU();
        TMatrix V = svd.matrixV();
        T det_uv = TMatrix(U*V.transpose()).determinant();
        TMatrix C = TMatrix::Identity(D, D);
        C(D-1, D-1) = det_uv;
        _paras._R = U * C * V.transpose();

        T s_upper = TMatrix(A.transpose()*_paras._R).trace();
        T s_lower = TMatrix(Y_hat.transpose()*this->_P1.asDiagonal()*Y_hat).trace();
        _paras._s =  s_upper / s_lower; 
            
        _paras._t = mu_x - _paras._s * _paras._R * mu_y;

        T tr_f = TMatrix(X_hat.transpose()*this->_PT1.asDiagonal()*X_hat).trace();
        T tr_b = TMatrix(A.transpose()*_paras._R).trace();
        _paras._sigma2 = (tr_f - _paras._s * tr_b) / (N_P * D);
        _paras._sigma2 = abs(_paras._sigma2);

    }

    template <typename T, int D>
    void CPDRigid<T, D>::align()
    {
        this->_T = (_paras._s) * (this->_model) * (_paras._R).transpose() + 
            TVector(this->_M).setOnes() * (_paras._t).transpose();
    }

    template <typename T, int D>
    T CPDRigid<T, D>::computeGaussianExp(size_t m, size_t n)
    {
        TVector vec = TVector(this->_T.row(m) - this->_data.row(n));
        T g_exp = exp(-vec.squaredNorm()/(2*_paras._sigma2));
        return g_exp;
    }

    template <typename T, int D>
    T CPDRigid<T, D>::energy() 
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
}
#endif
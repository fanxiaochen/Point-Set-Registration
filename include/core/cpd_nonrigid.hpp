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
        _type = NONRIGID;

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
        T e_tol = 10 + _e_tol;
        T e = 0;
        
        normalize();
        initialization();

        if (_vision)
        {
            RenderThread<T, D>::instance()->updateModel(_model);
            RenderThread<T, D>::instance()->updateData(_data);
            RenderThread<T, D>::instance()->startThread();
        }

        while (iter_num < _iter_num && e_tol > _e_tol && _paras._sigma2 > 10 * _v_tol)
        {

            e_step();
            
            T old_e = e;
            e = energy();
            e += _paras._lambda/2 * (_paras._W.transpose()*_G*_paras._W).trace();
            e_tol = abs((e - old_e) / e);

            m_step();

            if (_vision == true)
                RenderThread<T, D>::instance()->updateModel(_T);

            iter_num ++;	
        }
        
        correspondences();
        updateModel();
        denormalize();
        RenderThread<T, D>::instance()->cancel();	
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::correspondences()
    {
        _corres.setZero(_M, _N);

        for (size_t n = 0; n < _N; n ++)
        {
            typename std::vector<T> t_exp;
            T sum_exp = 0;
            T c = pow((2*M_PI*_paras._sigma2), 0.5*D) * (_w/(1-_w)) * (T(_M)/_N);
            for (size_t m = 0; m < _M; m ++)
            {
                T m_exp = computeGaussianExp(m, n);
                t_exp.push_back(m_exp);
                sum_exp += m_exp;
            }

            for (size_t m = 0; m < _M; m ++)
            {
                _corres(m, n) = t_exp.at(m) / (sum_exp + c);
            }
        }
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::initialization()
    {

        _paras._W = TMatrix::Zero(_M, D);

        T sigma_sum = _M*(_data.transpose()*_data).trace() + 
            _N*(_model.transpose()*_model).trace() - 
            2*(_data.colwise().sum())*(_model.colwise().sum()).transpose();
        _paras._sigma2 = sigma_sum / (D*_N*_M);

        initTransform();
        constructG();
        
        if (_lr)
            lr_approximate<T, D>(_G, _Q, _S, _K, _lr_maxitr);
    }

    template<typename T, int D>
    void CPDNRigid<T, D>::e_step()
    {
        if (!_fgt)
        {
            correspondences();
            _P1 = _corres * TVector(_N).setOnes();
            _PT1 = _corres.transpose() * TVector(_M).setOnes();
            _PX = _corres * _data;
        }
        else
        {
            T c = pow((2*M_PI*_paras._sigma2), 0.5*D) * (_w/(1-_w)) * (T(_M)/_N);
            TMatrix KT1 = fgt<T, D>(_T, _data, TVector(_M).setOnes(), sqrt(2*_paras._sigma2), _fgt_eps);
            TVector a = (TVector(KT1) + c*TVector(_N).setOnes()).cwiseInverse();

            TMatrix aX = TMatrix::Zero(_N, D);
            for (size_t i = 0; i < D; i ++)
            {
                aX.col(i) = _data.col(i).cwiseProduct(a);
            }

            _PT1 = TVector(_N).setOnes() - c * a;
            _P1 = fgt<T, D>(_data, _T, a, sqrt(2*_paras._sigma2), _fgt_eps);
            _PX = fgt<T, D>(_data, _T, aX, sqrt(2*_paras._sigma2), _fgt_eps);
        }
    }

    template<typename T, int D>
    void CPDNRigid<T, D>::m_step()
    {
        T N_P = _P1.sum();

        if (!_lr)
        {
            TMatrix A = (_P1.asDiagonal()*_G + _paras._lambda*_paras._sigma2*TMatrix::Identity(_M, _M));
            TMatrix B = _PX - _P1.asDiagonal() * _model;
            _paras._W = A.inverse() * B;
        }
        else
        {
            P1_correlation();
            TMatrix A1 = ((1/(_paras._lambda*_paras._sigma2))*_P1).asDiagonal();
            TMatrix A2 = _Q * (_S.inverse() + _Q.transpose()*A1*_Q).inverse() * _Q.transpose();
            TMatrix A_inv = A1 - A1 * A2 * A1;
            TMatrix B = _P1.cwiseInverse().asDiagonal() * _PX - _model;
            _paras._W = A_inv * B;
        }

        align();

        _paras._sigma2 = 1/(N_P*D) * ((_data.transpose()*_PT1.asDiagonal()*_data).trace() -
            2*(_PX.transpose()*_T).trace() + (_T.transpose()*_P1.asDiagonal()*_T).trace());

    }

    template<typename T, int D>
    void CPDNRigid<T, D>::align()
    {
        _T = _model + _G * _paras._W;
    }

    template <typename T, int D>
    T CPDNRigid<T, D>::energy()
    {
        T e = 0;
        
        for (size_t n = 0; n < _N; n ++)
        {
            T sp = 0;
            for (size_t m = 0; m < _M; m ++)
            {
                sp += computeGaussianExp(m, n);
            }

            sp += pow((2*M_PI*_paras._sigma2), 0.5*D) * (_w/(1-_w)) * (T(_M)/_N);

            e += -log(sp);

        }

        e += _N * D * log(_paras._sigma2) / 2;

        return e;
    }

    template <typename T, int D>
    T CPDNRigid<T, D>::computeGaussianExp(size_t m, size_t n)
    {
        TVector vec = TVector(_T.row(m) - _data.row(n));
        T g_exp = exp(-vec.squaredNorm()/(2*_paras._sigma2));
        return g_exp;
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::constructG()
    {
        _G = TMatrix::Zero(_M, _M);

        for (size_t i = 0; i < _M; i ++)
        {
            for (size_t j = 0; j < _M; j ++)
            {
                _G(i, j) = exp(-TVector(_model.row(i)-_model.row(j)).squaredNorm()/(2*_paras._beta*_paras._beta));
            }
        }
    }

    template <typename T, int D>
    void CPDNRigid<T, D>::P1_correlation()
    {
        T min_numerics = std::numeric_limits<T>::epsilon();

        for (size_t i = 0, i_end = _P1.rows(); i < i_end; i ++)
        {
            if (_P1(i, 0) < min_numerics)
                _P1(i, 0) = min_numerics;
        }
    }
}



#endif
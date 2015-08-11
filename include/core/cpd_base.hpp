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

#ifndef CPD_BASE_HPP
#define CPD_BASE_HPP

#include "base/parameters.hpp"
#include "core/registrator.hpp"

namespace cpd
{
    template <typename T, int D>
    class CPDBase: public Registrator<T, D>
    {
    public:
        CPDBase();
        virtual ~CPDBase();

        void setIterativeNumber(size_t iter_num);
        void setVarianceTolerance(T v_tol);
        void setEnergyTolerance(T e_tol);
        void setOutlierWeight(T w);
        void setFgtFlag(bool fgt);
        void setFgtEpsilon(T fgt_eps);
        void setLowRankFlag(bool lr);
        void setKLowRank(int K);
        void setLRMaxIteration(size_t lr_maxitr);

        void normalize();
        void denormalize();

        inline const TMatrix& getTransform(){ return _T; } 
        inline const TMatrix& getCorrespondences(){ return _corres; }

        virtual void run() = 0;

    private:
        virtual void initialization() = 0;
        virtual void e_step() = 0;
        virtual void m_step() = 0;
        virtual void align() = 0;
        virtual void correspondences() = 0;

    protected:
        void updateModel();
        void initTransform();

    protected:
        size_t      _iter_num;
        T           _v_tol;
        T           _e_tol;
        T           _w;

        TMatrix     _corres;
        TMatrix     _T;

        TMatrix     _P1;
        TMatrix     _PT1;
        TMatrix     _PX;

        bool        _fgt;
        T           _fgt_eps;

        bool        _lr;
        int         _K;
        size_t      _lr_maxitr;

        Normal<T, D>    _normal_model;
        Normal<T, D>    _normal_data;
    };
}

namespace cpd
{
    template <typename T, int D>
    CPDBase<T, D>::CPDBase()
        : _iter_num(50), _v_tol(1e-3), _e_tol(1e-3), _w(0),
        _fgt(false), _fgt_eps(1e-3), _lr(false), _K(0), _lr_maxitr(40)
    {}

    template <typename T, int D>
    CPDBase<T, D>::~CPDBase(){}

    template <typename T, int D>
    void CPDBase<T, D>::setIterativeNumber(size_t iter_num)
    {
        _iter_num = iter_num;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setVarianceTolerance(T v_tol)
    {
        _v_tol = v_tol;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setEnergyTolerance(T e_tol)
    {
        _e_tol = e_tol;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setOutlierWeight(T w)
    {
        _w = w;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setFgtFlag(bool fgt)
    {
        _fgt = fgt;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setFgtEpsilon(T fgt_eps)
    {
        _fgt_eps = fgt_eps;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setLowRankFlag(bool lr)
    {
        _lr = lr;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setKLowRank(int K)
    {
        _K = K;
    }

    template <typename T, int D>
    void CPDBase<T, D>::setLRMaxIteration(size_t lr_maxitr)
    {
        _lr_maxitr = lr_maxitr;
    }

    template <typename T, int D>
    void CPDBase<T, D>::updateModel()
    {
        this->_model = _T;
    }

    template <typename T, int D>
    void CPDBase<T, D>::initTransform()
    {
        _T = this->_model;
    }

    template <typename T, int D>
    void CPDBase<T, D>::normalize()
    {
        _normal_model._means = this->_model.colwise().mean();
        _normal_data._means = this->_data.colwise().mean();

        this->_model = this->_model - _normal_model._means.transpose().replicate(this->_M, 1);
        this->_data = this->_data - _normal_data._means.transpose().replicate(this->_N, 1);

        _normal_model._scale = sqrt(this->_model.array().square().sum() / this->_M);
        _normal_data._scale = sqrt(this->_data.array().square().sum() / this->_N);

        this->_model = this->_model / _normal_model._scale;
        this->_data = this->_data / _normal_data._scale;
    }

    template <typename T, int D>
    void CPDBase<T, D>::denormalize()
    {
        this->_model = this->_model * _normal_data._scale + _normal_data._means.transpose().replicate(this->_M, 1);
        this->_data = this->_data * _normal_data._scale + _normal_data._means.transpose().replicate(this->_N, 1);

    }
}

#endif
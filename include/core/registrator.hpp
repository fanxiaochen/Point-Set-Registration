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

#ifndef REGISTRATOR_HPP
#define REGISTRATOR_HPP

#include "base/data.hpp"

namespace cpd
{
    template <typename T, int D>
    class Registrator
    {
    public:
        Registrator();
        virtual ~Registrator();

        void setInputData(const DTMatrixD& model, const DTMatrixD& data);
        void setVision(bool vision);

        inline RegType getRegistrationType(){ return _type; }
        inline DTMatrixD& getModel(){ return _model; }
        inline DTMatrixD& getData(){ return _data; }

        virtual void run() = 0;

    protected:
        DTMatrixD            _model;
        DTMatrixD            _data;
        size_t              _M;
        size_t              _N;
        RegType             _type;
        bool                _vision;
    };
}

namespace cpd
{
    template <typename T, int D>
    Registrator<T, D>::Registrator()
        : _type(UNDEFINED), _vision(false), _M(0), _N(0)
    {

    }

    template <typename T, int D>
    Registrator<T, D>::~Registrator()
    {

    }

    template <typename T, int D>
    void Registrator<T, D>::setInputData(const DTMatrixD& model, const DTMatrixD& data)
    {
        _model = model;
        _data = data;

        // determine data number
        _M = _model.rows();
        _N = _data.rows();
    }

    template <typename T, int D>
    void Registrator<T, D>::setVision(bool vision)
    {
        _vision = vision;
    }
}

#endif
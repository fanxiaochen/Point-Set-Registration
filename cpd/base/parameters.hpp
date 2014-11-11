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

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "base/matrix.hpp"

namespace cpd
{
    template <typename T, int D>
    struct RigidParas  
    {
        TMatrix _R;
        TVector _t;
        T       _s;
        T       _sigma2;
    };

    template <typename T, int D>
    struct AffineParas  
    {
        TMatrix _B;
        TVector _t;
        T       _sigma2;
    };

    template <typename T, int D>
    struct NRigidParas
    {
        TMatrix _W;
        T       _sigma2;
        T       _lambda;
        T       _beta;
    };

    template <typename T, int D>
    struct Normal
    {
        TVector _means;
        T       _scale;
    };
}

#endif
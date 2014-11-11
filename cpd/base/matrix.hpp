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

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <Eigen/Core>

namespace cpd
{
    template <typename T, int D>
    struct MatrixType
    {
        typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
    
        typedef Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::RowMajor> MatrixD;

        typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
    };
    
#define     TVector             typename MatrixType<T, D>::Vector
#define     TMatrixD            typename MatrixType<T, D>::MatrixD
#define     TMatrix             typename MatrixType<T, D>::Matrix

}

#endif

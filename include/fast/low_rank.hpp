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

#ifndef LOW_RANK_HPP
#define LOW_RANK_HPP

#include <Eigen/Eigenvalues>
#include "base/matrix.hpp"

namespace cpd
{
    template <typename T, int D>
    struct EigenType
    {
        typedef typename Eigen::EigenSolver<TMatrix>::EigenvalueType EigenvalueType;

        typedef typename Eigen::EigenSolver<TMatrix>::EigenvectorsType EigenvectorsType;
    };
    

    template <typename T>
    struct EigenValue
    {
        EigenValue(){};

        EigenValue(T value, size_t idx)
        {
            _value = value;
            _idx = idx;
        }

        bool operator()(EigenValue i, EigenValue j)
        {
            return i._value > j._value;
        }

        T       _value;
        size_t  _idx;
    };


    template <typename T, int D>
    void k_extract(const typename EigenType<T, D>::EigenvalueType& eigen_values, 
        const typename EigenType<T, D>::EigenvectorsType& eigen_vectors, 
        TMatrix& Q, TMatrix& S, int K)
    {
        size_t eigen_num = eigen_values.rows();

        std::vector<EigenValue<T> > ev;

        for (size_t i = 0; i < eigen_num; i ++)
        {
            std::complex<T> cv = eigen_values(i);
            EigenValue<T> i_ev(cv.real(), i);
            ev.push_back(i_ev);
        }

        std::sort(ev.begin(), ev.end(), EigenValue<T>());

        int gm = eigen_vectors.rows();
        Q.resize(gm, K);
        TVector s(K);
        
        for (size_t i = 0; i < K; i ++)
        {
            s(i) = ev[i]._value;
            typename MatrixType<std::complex<T>, D>::Matrix q_ci = eigen_vectors.col(ev[i]._idx);
            for (size_t j = 0, j_end = q_ci.rows(); j < j_end; j ++)
            {
                Q(j, i) = q_ci(j).real();
            }
        }

        S = s.asDiagonal();
    }
    

    template <typename T, int D>
    void lr_approximate(const TMatrix& G, TMatrix& Q, TMatrix& S, int K, size_t lr_maxitr)
    {
        typename Eigen::EigenSolver<TMatrix> es;
        es.setMaxIterations(lr_maxitr*G.rows());
        es.compute(G);

        const typename EigenType<T, D>::EigenvalueType& eigen_values = es.eigenvalues();
        const typename EigenType<T, D>::EigenvectorsType& eigen_vectors = es.eigenvectors();	
        k_extract<T, D>(eigen_values, eigen_vectors, Q, S, K);
    }

    

}

#endif
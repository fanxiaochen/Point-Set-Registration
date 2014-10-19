#ifndef FGT_WRAPPER_HPP
#define FGT_WRAPPER_HPP

#define FIGTREE_NO_ANN 

#include "3rdparty/figtree/figtree.h"
#include "base/Matrix.hpp"

namespace cpd
{
	template <typename T, int D>
	TMatrix fgt(const TMatrix& x, const TMatrix& y, const TMatrix& q, T h, 
		T epsilon = 1e-3,
		int evalMethod = FIGTREE_EVAL_AUTO,
		int ifgtParamMethod = FIGTREE_PARAM_NON_UNIFORM,
		int ifgtTruncMethod = FIGTREE_TRUNC_CLUSTER,
		int verbose = 0)
	{
		typename MatrixType<double, D>::MatrixD x_r = x;
		typename MatrixType<double, D>::MatrixD y_r = y;
		typename MatrixType<double, D>::Matrix q_r = q.transpose();

		int d = D;
		int N = x_r.rows();
		int M = y_r.rows();
		int W = q_r.rows();

		double *X, *Y, *Q;
		X = x_r.data();
		Y = y_r.data();
		Q = q_r.data();

		typename MatrixType<double, D>::Matrix G(W, M);

		/*T *G = new T[W*M];
		memset(G, 0, sizeof(T)*W*M);*/

		figtree(d, N, M, W, X, h, Q, Y, epsilon, G.data(), evalMethod, ifgtParamMethod, ifgtTruncMethod, verbose);

		TMatrix g = G.transpose();
		//g.data() = &G;
		//std::cout << g << std::endl;
		return g;
	}
}

#endif
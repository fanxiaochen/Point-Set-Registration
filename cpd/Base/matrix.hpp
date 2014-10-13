#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <Eigen/Dense>

namespace cpd
{
	template <typename T, int D>
	struct MatrixType
	{
		typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	
		typedef Eigen::Matrix<T, Eigen::Dynamic, D> MatrixD;

		typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;

		typedef Eigen::DiagonalMatrix<T, D> DiagonalMatrix;
	};
	
#define		Vector			typename MatrixType<T, D>::Vector	
#define		MatrixD			typename MatrixType<T, D>::MatrixD	
#define		Matrix			typename MatrixType<T, D>::Matrix	
#define		DiagonalMatrix	typename MatrixType<T, D>::DiagonalMatrix;
}

#endif

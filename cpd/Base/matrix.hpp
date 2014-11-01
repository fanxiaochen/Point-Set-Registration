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
	
#define		TVector				typename MatrixType<T, D>::Vector	
#define		TMatrixD			typename MatrixType<T, D>::MatrixD	
#define		TMatrix				typename MatrixType<T, D>::Matrix

}

#endif

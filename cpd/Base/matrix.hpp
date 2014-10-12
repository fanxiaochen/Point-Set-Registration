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
	};
	
//#define	Vector	MatrixType<T, D>::Vector	
//#define	MatrixD MatrixType<T, D>::MatrixD	
//#define	Matrix	MatrixType<T, D>::Matrix	
}

#endif

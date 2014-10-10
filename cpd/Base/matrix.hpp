#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <Eigen/Dense>

namespace cpd
{
	typedef float value_type;

	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 1> Vector;
	
	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 2> MatrixM2;

	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 3> MatrixM3;

	typedef Eigen::Matrix<value_type, Eigen::Dynamic, 4> MatrixM4;

	typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> Matrix;
}

#endif

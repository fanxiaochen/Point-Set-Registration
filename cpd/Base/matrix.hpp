#ifndef MATRIX_H
#define MATRIX_H

#include <Eigen/Dense>

namespace cpd
{
	template <typename T, int M>
	class Matrix2: public Eigen::Matrix<T, M, 2>{};

	template <typename T, int M>
	class Matrix3: public Eigen::Matrix<T, M, 3>{};

	template <typename T, int M>
	class Matrix4: public Eigen::Matrix<T, M, 4>{};

	template <typename T, int M, int N>
	class MatrixN: public Eigen::Matrix<T, M, N>{};
}

#endif

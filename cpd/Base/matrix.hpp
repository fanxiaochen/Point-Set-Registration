#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <Eigen/Dense>

namespace cpd
{
	template <typename T>
	class MatrixM2: public Eigen::Matrix<T, Eigen::Dynamic, 2>{};

	template <typename T>
	class MatrixM3: public Eigen::Matrix<T, Eigen::Dynamic, 3>{};

	template <typename T>
	class Matrix2: public Eigen::Matrix<T, 2, 2>{};

	template <typename T>
	class Matrix3: public Eigen::Matrix<T, 3, 3>{};

	/*template <typename T, int M>
	class Matrix4: public Eigen::Matrix<T, M, 4>{};*/

	/*template <typename T>
	class Matrix: public Eigen::Matrix<T, Dynamic, Dynamic>{};*/
}

#endif

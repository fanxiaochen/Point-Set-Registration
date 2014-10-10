#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <Eigen/Dense>

namespace cpd
{
	template <typename T>
	class Vector2: public Eigen::Matrix<T, 2, 1>{};

	template <typename T>
	class Vector3: public Eigen::Matrix<T, 3, 1>{};

	template <typename T>
	class Vector4: public Eigen::Matrix<T, 4, 1>{};
}

#endif


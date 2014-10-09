#include <Eigen/Dense>

namespace cpd
{
	template <class T>
	class Matrix2: public Eigen::Matrix<T, 2, 2>{};

	template <class T>
	class Matrix3: public Eigen::Matrix<T, 3, 3>{};

	template <class T>
	class Matrix4: public Eigen::Matrix<T, 4, 4>{};
}


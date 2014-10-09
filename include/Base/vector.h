#include <Eigen/Dense>

namespace cpd
{
	template <class T>
	class Vector2: public Eigen::Matrix<T, 2, 1>{};

	template <class T>
	class Vector3: public Eigen::Matrix<T, 3, 1>{};

	template <class T>
	class Vector4: public Eigen::Matrix<T, 4, 1>{};
}


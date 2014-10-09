#include <iostream>
#include "Base/vector.h"
#include "Base/matrix.h"
#include "Base/types.h"

using namespace Eigen;
int main()
{
	cpd::Matrix2<cpd::value_type> m;
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	std::cout << "Here is the matrix m:\n" << m << std::endl;
	cpd::Vector2<float> v;
	v(0) = 4;
	v(1) = v(0) - 1;
	std::cout << "Here is the vector v:\n" << v << std::endl;
}
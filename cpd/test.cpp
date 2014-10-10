#include <iostream>
#include "Base/vector.hpp"
#include "Base/matrix.hpp"
#include "Base/data.hpp"
#include "Core/registrator.hpp"
//#include "Core/cpd_base.h"
//#include "Core/cpd_rigid.h"

int main()
{
	cpd::Matrix2<cpd::value_type, 2> m;
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	std::cout << "Here is the matrix m:\n" << m << std::endl;
	cpd::Vector2<float> v;
	v(0) = 4;
	v(1) = v(0) - 1;
	std::cout << "Here is the vector v:\n" << v << std::endl;

	cpd::Matrix2<cpd::value_type, 2> mat;
	cpd::Registrator<cpd::Matrix2<cpd::value_type, 2> > reg;
	reg.getCorrespondences();
	mat = reg.getData();
	/*cpd::Matrix2<cpd::value_type, 2> mat;
	cpd::CPDRigid<float >* cpd_base = new cpd::CPDRigid<float>;
	cpd_base->apply();*/
}
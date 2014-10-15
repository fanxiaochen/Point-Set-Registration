#include <iostream>
#include "Base/vector.hpp"
#include "Base/matrix.hpp"
#include "Base/data.hpp"
#include "Core/registrator.hpp"
//#include "Core/cpd_base.h"
#include "Core/cpd_rigid.hpp"

using namespace cpd;

int main()
{
	MatrixType<float, 3>::MatrixD model(4,3), data(4,3);
	model << 1,2,3,
			4,5,6,
			7,8,9,
			10,10,10;
	data << 1,2,3,
			4,5,6,
			7,8,9,
			10,10,10;

	Registrator<float, 3> reg;
	reg.setInputData(model, data);
	reg.setType(RIGID);
	reg.run();

	std::cout << "results:" << std::endl;
	std::cout << "model:" << std::endl;
	std::cout << reg.getModel() << std::endl;
	std::cout << "data:" << std::endl;
	std::cout << reg.getData() << std::endl;
}
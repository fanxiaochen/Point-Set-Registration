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
	MatrixType<float, 2>::MatrixD model(4,2), data(4,2);
	model << 1,2,
			4,5,
			7,8,
			10,10;
	data << 1,2,
			4,5,
			7,8,
			10,10;

	CPDRigid<float, 2>* reg = new CPDRigid<float, 2>();
	reg->setInputData(model, data);
	reg->setType(RIGID);
	reg->setIterativeNumber(150);
	reg->setMinimumValue();
	reg->setEnergyTolerance(1e-5);
	reg->setOutlierWeight(0.1);
	reg->run();

	std::cout << "results:" << std::endl;
	std::cout << "model:" << std::endl;
	std::cout << reg->getModel() << std::endl;
	std::cout << "data:" << std::endl;
	std::cout << reg->getData() << std::endl;
}
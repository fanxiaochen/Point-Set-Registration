#include <iostream>
#include "Base/vector.hpp"
#include "Base/matrix.hpp"
#include "Base/data.hpp"
#include "Core/registrator.hpp"
//#include "Core/cpd_base.h"
#include "Core/cpd_rigid.hpp"
#include "Core/cpd_nonrigid.hpp"

using namespace cpd;

int main()
{
	MatrixType<float, 2>::MatrixD model(91,2), data(91,2);
	/*model << 1,2,
			4,5,
			7,8,
			10,10;
	data << 1,2,
			4,5,
			7,8,
			10,10;
	data = - model;*/
	getInputData<float, 2>("x.txt", "y.txt", model, data);

	CPDNRigid<float, 2>* reg = new CPDNRigid<float, 2>();
	reg->setInputData(model, data);
	reg->setType(NONRIGID);
	reg->setVision(true);
	reg->setIterativeNumber(100);
	reg->setMinimumValue();
	reg->setEnergyTolerance(1e-8);
	reg->setOutlierWeight(0.1);
	reg->run();

	std::cout << "results:" << std::endl;
	std::cout << "model:" << std::endl;
	std::cout << reg->getModel() << std::endl;
	std::cout << "data:" << std::endl;
	std::cout << reg->getData() << std::endl;
}
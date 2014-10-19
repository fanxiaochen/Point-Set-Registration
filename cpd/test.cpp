#include <iostream>
#include "base/vector.hpp"
#include "base/matrix.hpp"
#include "base/data.hpp"
#include "core/registrator.hpp"
//#include "Core/cpd_base.h"
#include "core/cpd_rigid.hpp"
#include "core/cpd_nonrigid.hpp"

using namespace cpd;

int main()
{
	MatrixType<double, 2>::MatrixD model(91,2), data(91,2);
	//model << 1,2,
	//		4,5,
	//		7,8,
	//		10,10;
	//data << 1,2,
	//		4,5,
	//		7,8,
	//		10,10;
	////data = - model;
	getInputData<double, 2>("y-nonrigid.txt", "x-nonrigid.txt", model, data);

	CPDNRigid<double, 2>* reg = new CPDNRigid<double, 2>();
	reg->setInputData(model, data);
	reg->setVision(true);
	reg->setIterativeNumber(100);
	reg->setVarianceTolerance(1e-5);
	reg->setEnergyTolerance(1e-5);
	reg->setOutlierWeight(0.1);
	reg->setFgtFlag(true);
	reg->run();

	/*std::cout << "results:" << std::endl;
	std::cout << "model:" << std::endl;
	std::cout << reg->getModel() << std::endl;
	std::cout << "data:" << std::endl;
	std::cout << reg->getData() << std::endl;*/

	/*Eigen::Matrix<float, 3, 2, Eigen::RowMajor> a;
	Eigen::Matrix<float, 2, 3, Eigen::ColMajor> b;
	Eigen::Matrix<float, 2, 3, Eigen::RowMajor> c;

	a << 1,2,3,4,5,6;

	b << 1,2,
		3,4,
		5,6;

	c = b;

	c = Eigen::Matrix<float, 2, 3, Eigen::ColMajor>(c);

	std::cout << a << std::endl << std::endl;
	std::cout << b << std::endl << std::endl;
	std::cout << c << std::endl << std::endl;
	std::cout << *(a.data()+1) << std::endl << std::endl;
	std::cout << *(b.data()+1) << std::endl << std::endl;
	std::cout << a*b << std::endl << std::endl;
	std::cout << *(c.data()+1) << std::endl << std::endl;
	std::cout << *(Eigen::Matrix<float, 2, 3, Eigen::ColMajor>(c).data()+1) << std::endl << std::endl;*/

	/*double a[] = {0.1,0.1,0.2,0.3};
	float* b = (float*)a;

	std::cout << a[0] << std::endl;
	std::cout << b[0] << std::endl;*/
}
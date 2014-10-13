#ifndef DATA_HPP
#define DATA_HPP

#include <string>
#include "Base/matrix.hpp"

namespace cpd
{
	typedef enum {EMPTY, RIGID, AFFINE, NONRIGID} RegType;

	template <typename T, int D>
	void getInputData(const std::string& model_file, const std::string& data_file, 
		MatrixD& model, MatrixD& data)
	{
		// read files and fill the model and data matrice
		MatrixD matrix;
	}
}

#endif 
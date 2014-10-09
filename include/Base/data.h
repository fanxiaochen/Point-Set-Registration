#ifndef DATA_H
#define DATA_H

#include <string>
#include "Base/matrix.h"

namespace cpd
{
	typedef enum {RIGID, AFFINE, NONRIGID} RegType;

	typedef float value_type;

	template <typename T, int M>
	void getInputData(const std::string& model_file, const std::string& data_file, 
		Matrix2<T, M>& model, Matrix2<T, M>& data);

	template <typename T, int M>
	void getInputData(const std::string& model_file, const std::string& data_file, 
		Matrix3<T, M>& model, Matrix3<T, M>& data);	
}

#endif 
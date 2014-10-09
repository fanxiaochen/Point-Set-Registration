#include <string>
#include "Base/matrix.h"

namespace cpd
{
	typedef enum {RIGID, AFFINE, NONRIGID} RegType;

	template <class T>
	void getInputData(const std::string& model_file, const std::string& data_file, 
		Matrix2<T>& model, Matrix2<T>& data);

	template <class T>
	void getInputData(const std::string& model_file, const std::string& data_file, 
		Matrix3<T>& model, Matrix3<T>& data);	
}
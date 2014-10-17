#ifndef DATA_HPP
#define DATA_HPP

#include <fstream>
#include <string>
#include "Base/matrix.hpp"

namespace cpd
{
	typedef enum {UNDEFINED, RIGID, AFFINE, NONRIGID} RegType;

	template <typename T, int D>
	void getInputData(const std::string& model_file, const std::string& data_file, 
		TMatrixD& model, TMatrixD& data)
	{
		// read files and fill the model and data matrices
		std::fstream fin_m(model_file, std::ios_base::in), fin_d(data_file, std::ios_base::in);
		
		if (!fin_m || !fin_d)
		{
			std::cout << "cannot open the files!" << std::endl;
			exit(1);
		}
			

		T tmp = 0;
		size_t i = 0, j = 0;
		while (fin_m >> tmp)
		{
			model(i, j%D) = tmp;
			if (++j%D == 0)
			{
				++ i;
				j = 0;
			}
				
		}

		tmp = 0; 
		i = 0;
		j = 0;
		while (fin_d >> tmp)
		{
			data(i, j%D) = tmp;
			if (++j%D == 0)
			{
				++ i;
				j = 0;
			}
		}
	}
}

#endif 
#ifndef DATA_HPP
#define DATA_HPP

#include <fstream>
#include <string>
#include <queue>
#include "base/matrix.hpp"

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
			
		std::queue<T> m_values;
		T tmp = 0;
		size_t i = 0, j = 0;
		while (fin_m >> tmp)
		{
			/*model(i, j%D) = tmp;
			if (++j%D == 0)
			{
				++ i;
				j = 0;
			}*/
			m_values.push(tmp);
				
		}

		std::queue<T> d_values;
		tmp = 0; 
		i = 0;
		j = 0;
		while (fin_d >> tmp)
		{
			/*data(i, j%D) = tmp;
			if (++j%D == 0)
			{
				++ i;
				j = 0;
			}*/
			d_values.push(tmp);
		}

		size_t model_size = m_values.size() / D;
		size_t data_size = d_values.size() / D;

		if (model_size * D != m_values.size() ||
			data_size * D != d_values.size())
		{
			std::cout << "File is broken!" << std::endl;
		}

		model.resize(model_size, D);
		data.resize(data_size, D);

		for (size_t i = 0; i < model_size; i ++)
		{
			for (size_t j = 0; j < D; j ++)
			{
				model(i, j) = m_values.front();
				m_values.pop();
			}
		}

		for (size_t i = 0; i < data_size; i ++)
		{
			for (size_t j = 0; j < D; j ++)
			{
				data(i, j) = d_values.front();
				d_values.pop();
			}
		}
	}
}

#endif 
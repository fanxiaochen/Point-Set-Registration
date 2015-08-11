/**
 * @file
 * @author  Xiaochen Fan <fan.daybreak@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * 
 */

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
        DTMatrixD& model, DTMatrixD& data)
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
            m_values.push(tmp);		
        }

        std::queue<T> d_values;
        tmp = 0; 
        i = 0;
        j = 0;
        while (fin_d >> tmp)
        {
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
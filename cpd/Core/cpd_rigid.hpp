#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP
#include <cmath>
#include "Core/cpd_base.hpp"
#include "Base/parameters.hpp"

namespace cpd
{
	template <class T>
	class CPDRigid: public CPDBase<T>
	{
	public:
		CPDRigid();
		virtual ~CPDRigid();
		void apply();
	private:
		value_type computeGaussianExp(size_t m, size_t n);
		void e_step();
		void m_step();
	private:
		RigidParas	_paras;
		int			_dim;
		value_type	_w;
		Matrix		_corres;
	};
}

namespace cpd
{
	template <class T>
	CPDRigid<T>::CPDRigid(){}

	template <class T>
	CPDRigid<T>::~CPDRigid(){}

	template <class T>
	void CPDRigid<T>::apply()
	{
		Eigen::Matrix3f mat;
		mat.rows();

		// determine data dimension
		size_t model_rows = _model->rows();
		size_t model_cols = _model->cols();

		size_t data_rows = _data->rows();
		size_t data_cols = _data->cols();

		if (model_cols != data_cols)
		{
			std::cout << "the model and data are not in the same dimension!" << std::endl;
			return;
		}

		// initialization
		_dim = model_cols;
		_paras._R = Matrix::Identity(_dim, _dim);
		_paras._t = Vector::Zero(_dim, 1);
		_paras._s = 1;

		value_type sigma_sum = 0;
		for (size_t m = 0; m < model_rows; m ++)
		{
			Vector model_row = _model->row(m);
			for (size_t n = 0; n < data_rows; n ++)
			{
				Vector data_row = _data->row(n);
				sigma_sum += Vector(model_row - data_row).squaredNorm();
			}
		}
		_paras._squared_sigma = sigma_sum / (dim*model_rows*data_rows);

		_w = 0.5; // w means?


		//E-step

	}

	template <class T>
	void CPDRigid<T>::computeGaussianExp(size_t m, size_t n)
	{
		Vector vec = _paras._s*_paras._R*_model->row(m) - _data->row(n);
		value_type g_exp = exp(-vec.squaredNorm()/(2*_paras._squared_sigma));
		return g_exp;
	}

	template <class T>
	void CPDRigid<T>::e_step()
	{
		size_t model_rows = _model->rows();
		size_t data_rows = _data->rows();
		_corres.setZero(model_rows, data_rows);

		for (size_t n = 0; n < data_rows; n ++)
		{
			std::vector<value_type> t_exp;
			value_type sum_exp = 0;
			value_type c = pow((2*PI*_paras._squared_sigma), _dim/2) * (_w/(1-_w)) * (model_rows/data_rows);
			for (size_t m = 0; m < model_rows; m ++)
			{
				value_type m_exp = computeGaussianExp(m, n);
				t_exp.push_back(m_exp);
				sum_exp += m_exp;
			}

			for (size_t m = 0; m < model_rows; m ++)
			{
				_corres(m, n) = t_exp.at(m) / (sum_exp + c);
			}
		}
	}

	template <class T>
	void CPDRigid<T>::m_step()
	{

	}
}
#endif
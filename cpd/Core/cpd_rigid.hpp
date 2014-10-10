#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

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
		RigidParas _paras;
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
		const int dim = model_cols;
		_paras._R = Matrix::Identity(dim, dim);
		_paras._t = Vector::Zero(dim, 1);
		_paras._s = 1;

		value_type sigma_sum = 0;
		for (size_t i = 0; i < model_rows; i ++)
		{
			Vector model_row = _model->row(i);
			for (size_t j = 0; j < data_rows; j ++)
			{
				Vector data_row = _data->row(j);
				sigma_sum += Vector(model_row - data_row).squaredNorm();
			}
		}
		_paras._squared_sigma = sigma_sum / (dim*model_rows*data_rows);


	}
}
#endif
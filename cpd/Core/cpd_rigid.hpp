#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

#include "Core/cpd_base.hpp"

namespace cpd
{
	template <class T>
	class CPDRigid: public CPDBase<T>
	{
	public:
		CPDRigid();
		virtual ~CPDRigid();
		void apply();
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
		Matrix3<value_type> mat;
		mat.rows();


		size_t model_rows = _model->rows();
		size_t model_cols = _model->cols();

		size_t data_rows = _data->rows();
		size_t data_cols = _data->cols();

		if (model_cols != data_cols)
		{
			std::cout << "the model and data are not in the same dimension!" << std::endl;
			return;
		}

		const int R_m = model_cols;
		const int R_n = data_cols;
		const int m = 3; const int n = 3;
		Matrix3<value_type> R;
		


	}
}
#endif
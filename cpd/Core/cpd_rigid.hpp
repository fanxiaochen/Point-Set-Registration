#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP
#include <cmath>
#include <Eigen/SVD>
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
		size_t		_dim;
		size_t		_m;
		size_t		_n;
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
		_m = model_rows;
		_n = data_rows;

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
	value_type CPDRigid<T>::computeGaussianExp(size_t m, size_t n)
	{
		Vector vec = _paras._s*_paras._R*_model->row(m) - _data->row(n);
		value_type g_exp = exp(-vec.squaredNorm()/(2*_paras._squared_sigma));
		return g_exp;
	}

	template <class T>
	void CPDRigid<T>::e_step()
	{

		_corres.setZero(_m, _n);

		for (size_t n = 0; n < data_rows; n ++)
		{
			std::vector<value_type> t_exp;
			value_type sum_exp = 0;
			value_type c = pow((2*PI*_paras._squared_sigma), _dim/2) * (_w/(1-_w)) * (_m/_n);
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
		value_type N_P = _corres.sum();
		Vector mu_x = _data->transpose() * _corres.transpose() * Vector(_m).setOnes() / N_P;
		Vector mu_y = _model->transpose() * _corres * Vector(_n).setOnes() / N_P;
		T X_hat = _data - Vector(_n).setOnes() * mu_x.transpose();
		T Y_hat = _model - Vector(_m).setOnes() * mu_y.transpose();
		Matrix A = X_hat.transpose() * _corres.transpose() * Y_hat;

		JacobiSVD<Matrix> svd(A, ComputeThinU | ComputeThinV);
		Matrix U = svd.matrixU();
		Matrix V = svd.matrixV();
		value_type det_uv = Matrix(U*V.transpose()).determinant();
		Eigen::DiagonalMatrix<value_type, _dim> C;
		C.setIdentity();
		C(_dim - 1) = det_uv;

		_paras._R = U * C * V.transpose();
		_paras._s = Matrix(A.transpose()*_paras._R).trace() ;
		_paras._t = mu_x - _paras._s * _paras._R * mu_y;
		_paras._squared_sigma = ();


	}
}
#endif
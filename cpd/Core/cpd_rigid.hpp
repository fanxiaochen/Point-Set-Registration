#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

#include <cmath>
#include <Eigen/SVD>
#include "Core/cpd_base.hpp"
#include "Base/parameters.hpp"

namespace cpd
{
	template <typename T, int D>
	class CPDRigid: public CPDBase<T, D>
	{
	public:
		CPDRigid();
		virtual ~CPDRigid();
		void apply();
	private:
		T computeGaussianExp(size_t m, size_t n);
		void e_step();
		void m_step();
	private:
		RigidParas<T, D>	_paras;
		size_t				_M;
		size_t				_N;
		T					_w;
		Matrix				_corres;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDRigid<T, D>::CPDRigid(){}

	template <typename T, int D>
	CPDRigid<T, D>::~CPDRigid(){}

	template <typename T, int D>
	void CPDRigid<T, D>::apply()
	{
		Eigen::Matrix3f mat;
		mat.rows();

		// determine data dimension
		_M = _model->rows();
		_N = _data->rows();

		// initialization
		_paras._R = Matrix::Identity(D, D);
		_paras._t = Vector::Zero(D, D);
		_paras._s = 1;

		T sigma_sum = 0;
		for (size_t m = 0; m < _M; m ++)
		{
			Vector model_row = _model->row(m);
			for (size_t n = 0; n < _N; n ++)
			{
				Vector data_row = _data->row(n);
				sigma_sum += Vector(model_row - data_row).squaredNorm();
			}
		}
		_paras._squared_sigma = sigma_sum / (D*_M*_N);

		_w = 0.5; // w means?


		//E-step

	}

	template <typename T, int D>
	T CPDRigid<T, D>::computeGaussianExp(size_t m, size_t n)
	{
		Vector vec = _paras._s * _paras._R * _model->row(m) - _data->row(n);
		T g_exp = exp(-vec.squaredNorm()/(2*_paras._squared_sigma));
		return g_exp;
	}

	template <typename T, int D>
	void CPDRigid<T, D>::e_step()
	{

		_corres.setZero(_M, _N);

		for (size_t n = 0; n < _N; n ++)
		{
			std::vector<T> t_exp;
			T sum_exp = 0;
			T c = pow((2*PI*_paras._squared_sigma), D/2) * (_w/(1-_w)) * (_M/_N);
			for (size_t m = 0; m < _M; m ++)
			{
				T m_exp = computeGaussianExp(m, n);
				t_exp.push_back(m_exp);
				sum_exp += m_exp;
			}

			for (size_t m = 0; m < _M; m ++)
			{
				_corres(m, n) = t_exp.at(m) / (sum_exp + c);
			}
		}
	}

	template <typename T, int D>
	void CPDRigid<T, D>::m_step()
	{
		T N_P = _corres.sum();
		Vector mu_x = _data->transpose() * _corres.transpose() * Vector(_M).setOnes() / N_P;
		Vector mu_y = _model->transpose() * _corres * Vector(_N).setOnes() / N_P;
		T X_hat = _data - Vector(_N).setOnes() * mu_x.transpose();
		T Y_hat = _model - Vector(_M).setOnes() * mu_y.transpose();
		Matrix A = X_hat.transpose() * _corres.transpose() * Y_hat;

		JacobiSVD<Matrix> svd(A, ComputeThinU | ComputeThinV);
		Matrix U = svd.matrixU();
		Matrix V = svd.matrixV();
		T det_uv = Matrix(U*V.transpose()).determinant();
		Eigen::DiagonalMatrix<T, D> C;
		C.setIdentity();
		C(D - 1) = det_uv;

		_paras._R = U * C * V.transpose();
		_paras._s = Matrix(A.transpose()*_paras._R).trace() ;
		_paras._t = mu_x - _paras._s * _paras._R * mu_y;
		_paras._squared_sigma = ();

	}
}
#endif
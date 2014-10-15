#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

#include <cmath>
#include <vector>

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
		virtual void initialization();
		virtual void e_step();
		virtual void m_step();
		virtual void align();

	private:
		RigidParas<T, D>	_paras;
		size_t				_M;
		size_t				_N;
		T					_w;
		TMatrix				_corres;
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
		initialization();
		e_step();
		m_step();
		align();
	}

	template <typename T, int D>
	void CPDRigid<T, D>::initialization()
	{

		// determine data dimension
		_M = _model->rows();
		_N = _data->rows();

		// initialization
		_paras._R = TMatrix::Identity(D, D);
		_paras._t = TVector::Zero(D, 1);
		_paras._s = 1;

		T sigma_sum = 0;
		for (size_t m = 0; m < _M; m ++)
		{
			TVector model_row = _model->row(m);
			for (size_t n = 0; n < _N; n ++)
			{
				TVector data_row = _data->row(n);
				sigma_sum += TVector(model_row - data_row).squaredNorm();
			}
		}
		_paras._squared_sigma = sigma_sum / (D*_M*_N);

		_w = 0.5; // w means?

	}

	template <typename T, int D>
	T CPDRigid<T, D>::computeGaussianExp(size_t m, size_t n)
	{
		TVector vec = TVector(_paras._s * _paras._R * TVector(_model->row(m)) - TVector(_data->row(n)));
		T g_exp = exp(-vec.squaredNorm()/(2*_paras._squared_sigma));
		return g_exp;
	}

	template <typename T, int D>
	void CPDRigid<T, D>::e_step()
	{

		_corres.setZero(_M, _N);

		for (size_t n = 0; n < _N; n ++)
		{
			typename std::vector<T> t_exp;
			T sum_exp = 0;
			T c = pow((2*M_PI*_paras._squared_sigma), D/2) * (_w/(1-_w)) * (_M/_N);
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

		std::cout << _corres << std::endl;
	}

	template <typename T, int D>
	void CPDRigid<T, D>::m_step()
	{
		T N_P = _corres.sum();
		TVector mu_x = _data->transpose() * _corres.transpose() * TVector(_M).setOnes() / N_P;
		TVector mu_y = _model->transpose() * _corres * TVector(_N).setOnes() / N_P;
		TMatrixD X_hat = *_data - TMatrix(TVector(_N).setOnes() * mu_x.transpose());
		TMatrixD Y_hat = *_model - TMatrix(TVector(_M).setOnes() * mu_y.transpose());
		TMatrix A = X_hat.transpose() * _corres.transpose() * Y_hat;

		Eigen::JacobiSVD<TMatrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
		TMatrix U = svd.matrixU();
		TMatrix V = svd.matrixV();

		T det_uv = TMatrix(U*V.transpose()).determinant();
		TVector C(D);
		C.setIdentity();
		C(D-1) = det_uv;
		_paras._R = U * C.asDiagonal() * V.transpose();

		T s_upper = TMatrix(A.transpose()*_paras._R).trace();
		T s_lower = TMatrix(Y_hat.transpose()*
			TVector(_corres*TVector(_N).setOnes()).asDiagonal()*Y_hat).trace();
		_paras._s =  s_upper / s_lower; 
			
		_paras._t = mu_x - _paras._s * _paras._R * mu_y;

		T tr_f = TMatrix(X_hat.transpose()*
			TVector(_corres.transpose()*TVector(_M).setOnes()).asDiagonal()*X_hat).trace();
		T tr_b = TMatrix(A.transpose()*_paras._R).trace();
		_paras._squared_sigma = (tr_f - _paras._s * tr_b) / (N_P * D);

	}

	template <typename T, int D>
	void CPDRigid<T, D>::align()
	{
		*_model = (_paras._s) * (*_model) * (_paras._R).transpose() + 
			TVector(_M).setOnes() * (_paras._t).transpose();
	}
}
#endif
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

		void getCorrespondences();
		void getParameters();
		inline RegType getRegistrationType(){ return _type; }

		void run();

		void setScaler(bool scale);

	private:
		T computeGaussianExp(size_t m, size_t n);
		T energy();
		virtual void initialization();
		virtual void e_step();
		virtual void m_step();
		virtual void align();

	private:
		RigidParas<T, D>	_paras;
		size_t				_M;
		size_t				_N;
		TMatrix				_corres;
		bool				_scale;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDRigid<T, D>::CPDRigid(){}

	template <typename T, int D>
	CPDRigid<T, D>::~CPDRigid(){}

	template <typename T, int D>
	void CPDRigid<T, D>::run()
	{
		size_t iter_num = 0;
		T tol = 10 + _tol;
		T e = 0;

		initialization();
		while (iter_num < _iter_num && tol > _tol && _paras._sigma2 > 10 * _epsilon)
		{
			e_step();
			m_step();

			T e_new = energy();
			tol = (e_new - e) / e;

			iter_num ++;
		}
		
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
		_paras._sigma2 = sigma_sum / (D*_M*_N);

		_w = 0.5; 

	}

	template <typename T, int D>
	T CPDRigid<T, D>::computeGaussianExp(size_t m, size_t n)
	{
		TVector vec = TVector(_paras._s * _paras._R * TVector(_model->row(m)) - TVector(_data->row(n)));
		T g_exp = exp(-vec.squaredNorm()/(2*_paras._sigma2));
		return g_exp;
	}

	template <typename T, int D>
	T CPDRigid<T, D>::energy()
	{
		T e = 0;
		
		for (size_t n = 0; n < _N; n ++)
		{
			TVector d_vec(_data->row(n));
			for (size_t m = 0; m < _M; m ++)
			{
				TVector m_vec(_model->row(m));
				TVector T_m = (_paras._s) * (_paras._R) * m_vec + (_paras._t);
				
				e += TVector(d_vec - T_m).squaredNorm();
			}
		}

		e = e / (2*_paras._sigma2) + _corres.sum()*D*log(_paras._sigma2) / 2;

		return e;
	}

	template <typename T, int D>
	void CPDRigid<T, D>::e_step()
	{

		_corres.setZero(_M, _N);

		for (size_t n = 0; n < _N; n ++)
		{
			typename std::vector<T> t_exp;
			T sum_exp = 0;
			T c = pow((2*M_PI*_paras._sigma2), D/2) * (_w/(1-_w)) * (_M/_N);
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
		_paras._sigma2 = (tr_f - _paras._s * tr_b) / (N_P * D);

	}

	template <typename T, int D>
	void CPDRigid<T, D>::align()
	{
		*_model = (_paras._s) * (*_model) * (_paras._R).transpose() + 
			TVector(_M).setOnes() * (_paras._t).transpose();
	}

	template <typename T, int D>
	void CPDRigid<T, D>::getParameters()
	{

	}

	template <typename T, int D>
	void CPDRigid<T, D>::getCorrespondences()
	{

	}
}
#endif
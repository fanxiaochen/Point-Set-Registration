#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

#include <cmath>
#include <vector>

#include <Eigen/SVD>
#include "core/cpd_base.hpp"
#include "base/parameters.hpp"
#include "fast/fgt_wrapper.hpp"

namespace cpd
{
	template <typename T, int D>
	class CPDRigid: public CPDBase<T, D>
	{
	public:
		CPDRigid();
		virtual ~CPDRigid();

		inline RigidParas<T, D>& getParameters(){ return _paras; }

		void run();

	private:
		void initialization();
		void e_step();
		void m_step();
		void align();
		void correspondences();

		T computeGaussianExp(size_t m, size_t n);
		T energy();

	private:
		RigidParas<T, D>	_paras;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDRigid<T, D>::CPDRigid()
	{
		_type = RIGID;
	}

	template <typename T, int D>
	CPDRigid<T, D>::~CPDRigid(){}

	template <typename T, int D>
	void CPDRigid<T, D>::run()
	{
		size_t iter_num = 0;
		T e_tol = 10 + _e_tol;
		T e = 0;
		
		initialization();

		if (_vision)
		{
			RenderThread<T, D>::instance()->updateModel(_model);
			RenderThread<T, D>::instance()->updateData(_data);
			RenderThread<T, D>::instance()->startThread();
		}

		while (iter_num < _iter_num && e_tol > _e_tol && _paras._sigma2 > 10 * _v_tol)
		{
			e_step();

			T old_e = e;
			//std::cout << _corres << std::endl;
			e = energy();
			e_tol = abs((e - old_e) / e);

			m_step();
			align();

			if (_vision == true)
				RenderThread<T, D>::instance()->updateModel(_T);
			//std::cout << "iter = " << iter_num << " e_tol = " << e_tol << " sigma2 = " << _paras._sigma2 << std::endl;

			iter_num ++;	
		}
		/*std::cout << "lastiter:" << iter_num << std::endl;
		std::cout << "lasttol:" << e_tol << std::endl;
		std::cout << "lastsigma:" << _paras._sigma2 << std::endl;*/
		
		updateModel();
		RenderThread<T, D>::instance()->cancel();
		/*if (_vision == true)
		{
			vis = new Visualizer<T, D>();
			vis->updateModel(_T);
			vis->updateData(_data);
			vis->show();
		}*/
		
	}

	template <typename T, int D>
	void CPDRigid<T, D>::initialization()
	{

		// determine data number
		_M = _model.rows();
		_N = _data.rows();

		// initialization
		_paras._R = TMatrix::Identity(D, D);
		_paras._t = TVector::Zero(D, 1);
		_paras._s = 1;

		T sigma_sum = _M*(_data.transpose()*_data).trace() + 
			_N*(_model.transpose()*_model).trace() - 
			2*(_data.colwise().sum())*(_model.colwise().sum()).transpose();
		_paras._sigma2 = sigma_sum / (D*_N*_M);

		initTransform();
	}

	

	template <typename T, int D>
	void CPDRigid<T, D>::correspondences()
	{
		_corres.setZero(_M, _N);

		for (size_t n = 0; n < _N; n ++)
		{
			typename std::vector<T> t_exp;
			T sum_exp = 0;
			T c = pow((2*M_PI*_paras._sigma2), 0.5*D) * (_w/(1-_w)) * (_M/_N);
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
	void CPDRigid<T, D>::e_step()
	{
		if (!_fgt)
		{
			correspondences();
			_P1 = _corres * TVector(_N).setOnes();
			_PT1 = _corres.transpose() * TVector(_M).setOnes();
			_PX = _corres * _data;

		}
		else
		{
			T c = pow((2*M_PI*_paras._sigma2), 0.5*D) * (_w/(1-_w)) * (_M/_N);
			TMatrix KT1 = fgt<T, D>(_T, _data, TVector(_M).setOnes(), sqrt(2*_paras._sigma2));
			TVector a = (TVector(KT1) + c*TVector(_N).setOnes()).cwiseInverse();

			TMatrix aX = TMatrix::Zero(_N, D);
			for (size_t i = 0; i < D; i ++)
			{
				aX.col(i) = _data.col(i).cwiseProduct(a);
			}

			_PT1 = TVector(_N).setOnes() - c * a;
			_P1 = fgt<T, D>(_data, _T, a, sqrt(2*_paras._sigma2));
			_PX = fgt<T, D>(_data, _T, aX, sqrt(2*_paras._sigma2));

			//std::cout << "_P1:" << std::endl << _P1 << std::endl;
			////std::cout << "_PT1:" << std::endl << _PT1 << std::endl;
			//std::cout << "_PX:" << std::endl << _PX << std::endl;
		}
	}

	template <typename T, int D>
	void CPDRigid<T, D>::m_step()
	{
		T N_P = _P1.sum();

		//std::cout << energy() << std::endl;
		/*std::cout << _P1 << std::endl;
		std::cout << _PT1 << std::endl;*/

		TVector mu_x = _data.transpose() * _PT1 / N_P;
		TVector mu_y = _model.transpose() * _P1 / N_P;

		/*std::cout << mu_x << std::endl;
		std::cout << mu_y << std::endl;*/

		TMatrixD X_hat = _data - TMatrix(TVector(_N).setOnes() * mu_x.transpose());
		TMatrixD Y_hat = _model - TMatrix(TVector(_M).setOnes() * mu_y.transpose());
		//TMatrix A = X_hat.transpose() * _corres.transpose() * Y_hat;

		TMatrix A = (_PX-_P1*mu_x.transpose()).transpose() * 
			(_model - TMatrix(TVector(_M).setOnes() * mu_y.transpose()));

		//std::cout << A << std::endl;

		Eigen::JacobiSVD<TMatrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
		TMatrix U = svd.matrixU();
		TMatrix V = svd.matrixV();

		T det_uv = TMatrix(U*V.transpose()).determinant();
		TVector C(D);
		C.setIdentity();
		C(D-1) = det_uv;
		_paras._R = U * C.asDiagonal() * V.transpose();

		T s_upper = TMatrix(A.transpose()*_paras._R).trace();
		T s_lower = TMatrix(Y_hat.transpose()*_P1.asDiagonal()*Y_hat).trace();
		_paras._s =  s_upper / s_lower; 
			
		_paras._t = mu_x - _paras._s * _paras._R * mu_y;

		T tr_f = TMatrix(X_hat.transpose()*_PT1.asDiagonal()*X_hat).trace();
		T tr_b = TMatrix(A.transpose()*_paras._R).trace();
		_paras._sigma2 = (tr_f - _paras._s * tr_b) / (N_P * D);

		/*std::cout << std::endl;
		std::cout << _paras._R << std::endl;
		std::cout << _paras._t << std::endl;
		std::cout << _paras._s << std::endl;*/

	}

	template <typename T, int D>
	void CPDRigid<T, D>::align()
	{
		_T = (_paras._s) * (_model) * (_paras._R).transpose() + 
			TVector(_M).setOnes() * (_paras._t).transpose();

		//std::cout << *_model << std::endl;
	}

	template <typename T, int D>
	T CPDRigid<T, D>::computeGaussianExp(size_t m, size_t n)
	{
		TVector vec = TVector(_T.row(m) - _data.row(n));
		T g_exp = exp(-vec.squaredNorm()/(2*_paras._sigma2));
		return g_exp;
	}

	template <typename T, int D>
	T CPDRigid<T, D>::energy() // error occurs
	{
		T e = 0;
		
		for (size_t n = 0; n < _N; n ++)
		{
			T sp = 0;
			for (size_t m = 0; m < _M; m ++)
			{
				sp += computeGaussianExp(m, n);
			}

			sp += pow((2*M_PI*_paras._sigma2), 0.5*D) * (_w/(1-_w)) * (_M/_N);

			e += -log(sp);

		}

		e += _N * D * log(_paras._sigma2) / 2;

		return e;
	}
}
#endif
#ifndef CPD_NRIGID_HPP
#define CPD_NRIGID_HPP

#include <cmath>
#include <vector>

#include <Eigen/SVD>
#include "core/cpd_base.hpp"
#include "base/parameters.hpp"
#include "base/visualizer.hpp"

namespace cpd
{
	template <typename T, int D>
	class CPDNRigid: public CPDBase<T, D>
	{
	public:
		CPDNRigid();
		virtual ~CPDNRigid();

		void setLamda(T lamda);
		void setBeta(T beta);
		inline NRigidParas<T, D>& getParameters(){ return _paras; }

		inline TMatrix& getG(){ return _G; }

		void run();

	private:
		void initialization();
		void e_step();
		void m_step();
		void align();
		void correspondences();

		void constructG();
		T computeGaussianExp(size_t m, size_t n);
		T energy();

	private:
		NRigidParas<T, D>	_paras;
		TMatrix				_G;
		TMatrix				_Q;
		TMatrix				_S;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDNRigid<T, D>::CPDNRigid()
	{
		_type = NONRIGID;
	}

	template <typename T, int D>
	CPDNRigid<T, D>::~CPDNRigid(){}

	template <typename T, int D>
	void CPDNRigid<T, D>::setLamda(T lamda)
	{
		_paras._lamda = lamda;
	}

	template <typename T, int D>
	void CPDNRigid<T, D>::setBeta(T beta)
	{
		_paras._beta = beta;
	}

	template <typename T, int D>
	void CPDNRigid<T, D>::run()
	{
		Visualizer<T, D>* vis;

		size_t iter_num = 0;
		T e_tol = 10 + _e_tol;
		T e = 0;
		
		initialization();

		while (iter_num < _iter_num && e_tol > _e_tol && _paras._sigma2 > 10 * _v_tol)
		{
			e_step();
			m_step();

			T old_e = e;
			//std::cout << _corres << std::endl;

			e = energy();
			e_tol = abs((e - old_e) / e);

			std::cout << "iter = " << iter_num << " e_tol = " << e_tol << " sigma2 = " << _paras._sigma2 << std::endl;

			/*if (_vision == true)
				vis->updateModel(_T)*/;

			iter_num ++;	
		}
		std::cout << "lastiter:" << iter_num << std::endl;
		std::cout << "lasttol:" << e_tol << std::endl;
		std::cout << "lastsigma:" << _paras._sigma2 << std::endl;
		
		updateModel();

		if (_vision == true)
		{
			vis = new Visualizer<T, D>();
			vis->updateModel(_T);
			vis->updateData(_data);
			vis->show();
		}
		
	}

	template <typename T, int D>
	void CPDNRigid<T, D>::correspondences()
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
	void CPDNRigid<T, D>::initialization()
	{
		// determine data number
		_M = _model.rows();
		_N = _data.rows();

		_paras._W = TMatrix::Zero(_M, D);

		/*T sigma_sum = 0;
		for (size_t m = 0; m < _M; m ++)
		{
			TVector model_row = _model.row(m);
			for (size_t n = 0; n < _N; n ++)
			{
				TVector data_row = _data.row(n);
				sigma_sum += TVector(model_row - data_row).squaredNorm();
			}
		}
		_paras._sigma2 = sigma_sum / (D*_M*_N);*/

		T sigma_sum = _M*(_data.transpose()*_data).trace() + 
			_N*(_model.transpose()*_model).trace() - 
			2*(_data.colwise().sum())*(_model.colwise().sum()).transpose();
		_paras._sigma2 = sigma_sum / (D*_N*_M);

		_paras._lamda = 2;
		_paras._beta = 2;

		initTransform();
		constructG();
		
		if (_lr)
			lr_approximate<T, D>(_G, _Q, _S, _K);
	}

	template<typename T, int D>
	void CPDNRigid<T, D>::e_step()
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
		}
	}

	template<typename T, int D>
	void CPDNRigid<T, D>::m_step()
	{
		T N_P = _P1.sum();

		if (!_lr)
		{
			TMatrix A = (_P1.asDiagonal()*_G + _paras._lamda*_paras._sigma2*TMatrix::Identity(_M, _M));
			TMatrix B = _PX - _P1.asDiagonal() * _model;
			_paras._W = A.inverse() * B;
		}
		else
		{
			TMatrix A1 = ((1/(_paras._lamda*_paras._sigma2))*_P1).asDiagonal();
			TMatrix A2 = _Q * (_S.inverse() + _Q.transpose()*A1*_Q).inverse() * _Q.transpose();
			TMatrix A_inv = A1 - A1 * A2 * A1;
			TMatrix B = _P1.cwiseInverse().asDiagonal() * _PX - _model;
			_paras._W = A_inv * B;
		}

		align();
	
		_paras._sigma2 = 1/(N_P*D) * ((_data.transpose()*_PT1.asDiagonal()*_data).trace() -
			2*(_PX.transpose()*_T).trace() + (_T.transpose()*_P1.asDiagonal()*_T).trace());

	}

	template<typename T, int D>
	void CPDNRigid<T, D>::align()
	{
		_T = _model + _G * _paras._W;
	}

	template <typename T, int D>
	T CPDNRigid<T, D>::energy()
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

	template <typename T, int D>
	T CPDNRigid<T, D>::computeGaussianExp(size_t m, size_t n)
	{
		TVector vec = TVector(_T.row(m) - _data.row(n));
		T g_exp = exp(-vec.squaredNorm()/(2*_paras._sigma2));
		return g_exp;
	}

	template <typename T, int D>
	void CPDNRigid<T, D>::constructG()
	{
		_G = TMatrix::Zero(_M, _M);

		for (size_t i = 0; i < _M; i ++)
		{
			for (size_t j = 0; j < _M; j ++)
			{
				_G(i, j) = exp(-TVector(_model.row(i)-_model.row(j)).squaredNorm()/(2*_paras._beta*_paras._beta));
			}
		}
	}
}

#endif
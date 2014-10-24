#ifndef CPD_BASE_HPP
#define CPD_BASE_HPP

#include "base/parameters.hpp"

namespace cpd
{
	template <typename T, int D>
	class CPDBase: public Registrator<T, D>
	{
	public:
		CPDBase();
		virtual ~CPDBase();

		void setIterativeNumber(size_t iter_num);
		void setVarianceTolerance(T v_tol);
		void setEnergyTolerance(T e_tol);
		void setOutlierWeight(T w);
		void setFgtFlag(bool fgt);
		void setLowRankFlag(bool lr);
		void setKLowRank(int K);

		void normalize();
		void denormalize();

		inline const TMatrix& getTransform(){ return _T; } 
		inline const TMatrix& getCorrespondences(){ return _corres; }

		virtual void run() = 0;

	private:
		virtual void initialization() = 0;
		virtual void e_step() = 0;
		virtual void m_step() = 0;
		virtual void align() = 0;
		virtual void correspondences() = 0;

	protected:
		void updateModel();
		void initTransform();

	protected:
		size_t		_iter_num;
		T			_v_tol;
		T			_e_tol;
		T			_w;

		TMatrix		_corres;
		TMatrix		_T;

		TMatrix		_P1;
		TMatrix		_PT1;
		TMatrix		_PX;

		bool		_fgt;
		bool		_lr;
		int			_K;

		Normal<T, D>	_normal_model;
		Normal<T, D>	_normal_data;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDBase<T, D>::CPDBase()
		: _iter_num(0), _v_tol(0), _e_tol(0), _w(0),
		_fgt(false), _lr(false), _K(0)
	{}

	template <typename T, int D>
	CPDBase<T, D>::~CPDBase(){}

	template <typename T, int D>
	void CPDBase<T, D>::setIterativeNumber(size_t iter_num)
	{
		_iter_num = iter_num;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setVarianceTolerance(T v_tol)
	{
		_v_tol = v_tol;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setEnergyTolerance(T e_tol)
	{
		_e_tol = e_tol;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setOutlierWeight(T w)
	{
		_w = w;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setFgtFlag(bool fgt)
	{
		_fgt = fgt;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setLowRankFlag(bool lr)
	{
		_lr = lr;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setKLowRank(int K)
	{
		_K = K;
	}

	template <typename T, int D>
	void CPDBase<T, D>::updateModel()
	{
		_model = _T;
	}

	template <typename T, int D>
	void CPDBase<T, D>::initTransform()
	{
		_T = _model;
	}

	template <typename T, int D>
	void CPDBase<T, D>::normalize()
	{
		_normal_model._means = _model.colwise().mean();
		_normal_data._means = _data.colwise().mean();

		_model = _model - _normal_model._means.transpose().replicate(_M, 1);
		_data = _data - _normal_data._means.transpose().replicate(_N, 1);

		_normal_model._scale = sqrt(_model.array().square().sum() / _M);
		_normal_data._scale = sqrt(_data.array().square().sum() / _N);

		_model = _model / _normal_model._scale;
		_data = _data / _normal_data._scale;
	}

	template <typename T, int D>
	void CPDBase<T, D>::denormalize()
	{
		_model = _model * _normal_model._scale + _normal_model._means.transpose().replicate(_M, 1);
		_data = _data * _normal_data._scale + _normal_data._means.transpose().replicate(_N, 1);

	}
}

#endif
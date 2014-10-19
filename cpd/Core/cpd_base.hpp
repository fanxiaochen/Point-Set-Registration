#ifndef CPD_BASE_HPP
#define CPD_BASE_HPP

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

		size_t		_M;
		size_t		_N;
		TMatrix		_corres;
		TMatrix		_T;

		TMatrix		_P1;
		TMatrix		_PT1;
		TMatrix		_PX;

		bool		_fgt;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDBase<T, D>::CPDBase()
		: _iter_num(0), _v_tol(0), _e_tol(0), _w(0),
		_M(0), _N(0), _fgt(false)
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
	void CPDBase<T, D>::updateModel()
	{
		_model = _T;
	}

	template <typename T, int D>
	void CPDBase<T, D>::initTransform()
	{
		_T = _model;
	}
}

#endif
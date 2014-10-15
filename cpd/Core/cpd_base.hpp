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
		void setMinimumValue();
		void setEnergyTolerance(T tol);
		void setOutlierWeight(T w);

		virtual void getCorrespondences() = 0;
		virtual void getParameters() = 0;

		virtual void run() = 0;

	private:
		virtual void initialization() = 0;
		virtual void e_step() = 0;
		virtual void m_step() = 0;
		virtual void align() = 0;

	protected:
		size_t _iter_num;
		T _epsilon;
		T _tol;
		T _w;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDBase<T, D>::CPDBase()
		: _iter_num(0), _epsilon(0), _tol(0), _w(0)
	{}

	template <typename T, int D>
	CPDBase<T, D>::~CPDBase(){}

	template <typename T, int D>
	void CPDBase<T, D>::setIterativeNumber(size_t iter_num)
	{
		_iter_num = iter_num;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setMinimumValue()
	{
		_epsilon = std::numeric_limits<T>::epsilon();
	}

	template <typename T, int D>
	void CPDBase<T, D>::setEnergyTolerance(T tol)
	{
		_tol = tol;
	}

	template <typename T, int D>
	void CPDBase<T, D>::setOutlierWeight(T w)
	{
		_w = w;
	}
}

#endif
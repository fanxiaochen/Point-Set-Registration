#ifndef CPD_BASE_HPP
#define CPD_BASE_HPP

namespace cpd
{
	template <typename T, int D>
	class CPDBase
	{
	public:
		CPDBase();
		virtual ~CPDBase();

		void setInputData(TMatrixD* const model, TMatrixD* const data);
		virtual void apply() = 0;

	private:
		virtual void initialization() = 0;
		virtual void e_step() = 0;
		virtual void m_step() = 0;
		virtual void align() = 0;

	protected:
		TMatrixD* _model;
		TMatrixD* _data;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDBase<T, D>::CPDBase(){}

	template <typename T, int D>
	CPDBase<T, D>::~CPDBase(){}

	template <typename T, int D>
	void CPDBase<T, D>::setInputData(TMatrixD* const model, TMatrixD* const data)
	{
		_model = model;
		_data = data;
	}
}

#endif
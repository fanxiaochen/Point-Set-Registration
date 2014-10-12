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

		void setInputData(MatrixD* const model, MatrixD* const data);
		virtual void apply() = 0;

	/*private:
		virtual void intialization() = 0;
		virtual void em() = 0;
		virtual void align() = 0;*/

	protected:
		MatrixD* _model;
		MatrixD* _data;
	};
}

namespace cpd
{
	template <typename T, int D>
	CPDBase<T, D>::CPDBase(){}

	template <typename T, int D>
	CPDBase<T, D>::~CPDBase(){}

	template <typename T, int D>
	void CPDBase<T, D>::setInputData(MatrixD* const model, MatrixD* const data)
	{
		_model = model;
		_data = data;
	}
}

#endif
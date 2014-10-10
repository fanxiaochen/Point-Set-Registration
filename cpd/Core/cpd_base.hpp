#ifndef CPD_BASE_HPP
#define CPD_BASE_HPP

namespace cpd
{
	template <class T>
	class CPDBase
	{
	public:
		CPDBase();
		virtual ~CPDBase();

		void setInputData(T* const model, T* const data);
		virtual void apply() = 0;

	private:
		virtual void intialization() = 0;
		virtual void em() = 0;
		virtual void align() = 0;

	private:
		T* _model;
		T* _data;
	};
}

namespace cpd
{
	template <class T>
	CPDBase<T>::CPDBase(){}

	template <class T>
	CPDBase<T>::~CPDBase(){}

	template <class T>
	void CPDBase<T>::setInputData(T* const model, T* const data)
	{
		_model = model;
		_data = data;
	}
}

#endif
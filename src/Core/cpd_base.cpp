#include "Core/cpd_base.h"

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

	template <class T>
	void CPDBase<T>::apply()
	{
		intialization();
		em();
		align();
	}
}
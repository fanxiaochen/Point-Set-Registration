#include "Core/registrator.h"

namespace cpd
{
	template <class T>
	Registrator<T>::Registrator()
		:_cpd_base(NULL), RegType(0)
	{

	}

	template <class T>
	Registrator<T>::~Registrator()
	{

	}

	template <class T>
	void Registrator<T>::setInputData(const T& model, const T& data)
	{
		
	}

	template <class T>
	void Registrator<T>::setType(RegType type)
	{
		_type = type;
	}

	template <class T>
	void Registrator<T>::getCorrespondences()
	{

	}

	template <class T>
	T& Registrator<T>::getModel()
	{
		return _cpd_base->getModel();
	}

	template <class T>
	T& Registrator<T>::getData()
	{
		return _cpd_base->getData();
	}

	template <class T>
	void Registrator<T>::run()
	{
		if (_type == RIGID)

	}

	template <class T>
	void Registrator<T>::intialization()
	{

	}

	template <class T>
	void Registrator<T>::em()
	{

	}

	template <class T>
	void Registrator<T>::align()
	{

	}
}

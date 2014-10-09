#include "Core/registrator.h"

namespace cpd
{
	template <class T>
	Registrator<T>::Registrator()
	{

	}

	template <class T>
	Registrator<T>::~Registrator()
	{

	}

	template <class T>
	void Registrator<T>::setInputData(const T& model, const T& data)
	{
		_model = model;
		_data = data;
	}

	template <class T>
	void Registrator<T>::getCorrespondences()
	{

	}

	template <class T>
	void Registrator<T>::run()
	{

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

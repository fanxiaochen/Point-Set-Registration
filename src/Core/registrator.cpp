#include "Core/registrator.h"

namespace cpd
{
	template <class T>
	Registrator<T>::Registrator()
		:RegType(-1)
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
	void Registrator<T>::run()
	{
		if (_type == -1)
			std::cout << "Please set the type of the registration!" << std::endl;
		else if (_type == RIGID)
			rigid();
		else if (_type == AFFINE)
			affine();
		else if (_type == NONRIGID)
			nonrigid();
		else
			std::cout << "Please check your registration setting!" << std::endl;
	}

	template <class T>
	void Registrator<T>::rigid()
	{

	}

	template <class T>
	void Registrator<T>::affine()
	{

	}

	template <class T>
	void Registrator<T>::nonrigid()
	{

	}
}

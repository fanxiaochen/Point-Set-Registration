#ifndef REGISTRATOR_HPP
#define REGISTRATOR_HPP

#include "Base/data.hpp"

namespace cpd
{
	template <class T>
	class Registrator
	{
	public:
		static bool ONE_STEP;

	public:
		Registrator();
		~Registrator();

		void setInputData(const T& model, const T& data);
		void setType(RegType type);

		inline T& getModel(){return _model;}
		inline T& getData(){return _data;}

		void getCorrespondences();
		void getParameters();
		
		void run();

	private:
		void rigid();
		void affine();
		void nonrigid();

	private:
		T			_model;
		T			_data;
		RegType		_type;
	};
}

namespace cpd
{
	template <class T>
	Registrator<T>::Registrator()
		:_type(EMPTY)
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
	void Registrator<T>::getParameters()
	{

	}

	template <class T>
	void Registrator<T>::getCorrespondences()
	{

	}

	template <class T>
	void Registrator<T>::run()
	{
		if (_type == EMPTY)
			std::cout << "Please set the type of the registration!" << std::endl;
		else if (_type == RIGID)
			rigid();
		else if (_type == AFFINE)
			affine();
		else if (_type == NONRIGID)
			nonrigid();
		else
			std::cout << "Please check your registration type setting!" << std::endl;
	}

	template <class T>
	void Registrator<T>::rigid()
	{
		CPDRigid cpd;
		cpd.setInputData(&_model, &_data);
		cpd.apply();
	}

	template <class T>
	void Registrator<T>::affine()
	{
		CPDAffine cpd;
		cpd.setInputData(&_model, &_data);
		cpd.apply();
	}

	template <class T>
	void Registrator<T>::nonrigid()
	{
		CPDNRigid cpd;
		cpd.setInputData(&_model, &_data);
		cpd.apply();
	}
}

#endif
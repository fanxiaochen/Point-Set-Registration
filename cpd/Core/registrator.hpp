#ifndef REGISTRATOR_HPP
#define REGISTRATOR_HPP

#include "Base/data.hpp"

namespace cpd
{
	template <typename T, int D>
	class Registrator
	{
	public:
		static bool ONE_STEP;

	public:
		Registrator();
		~Registrator();

		void setInputData(const MatrixD& model, const MatrixD& data);
		void setType(RegType type);

		inline MatrixD& getModel(){return _model;}
		inline MatrixD& getData(){return _data;}

		void getCorrespondences();
		void getParameters();
		
		void run();

	private:
		void rigid();
		void affine();
		void nonrigid();

	private:
		MatrixD			_model;
		MatrixD			_data;
		RegType			_type;
	};
}

namespace cpd
{
	template <typename T, int D>
	Registrator<T, D>::Registrator()
		:_type(EMPTY)
	{

	}

	template <typename T, int D>
	Registrator<T, D>::~Registrator()
	{

	}

	template <typename T, int D>
	void Registrator<T, D>::setInputData(const MatrixD& model, const MatrixD& data)
	{
		_model = model;
		_data = data;
	}

	template <typename T, int D>
	void Registrator<T, D>::setType(RegType type)
	{
		_type = type;
	}

	template <typename T, int D>
	void Registrator<T, D>::getParameters()
	{

	}

	template <typename T, int D>
	void Registrator<T, D>::getCorrespondences()
	{

	}

	template <typename T, int D>
	void Registrator<T, D>::run()
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

	template <typename T, int D>
	void Registrator<T, D>::rigid()
	{
		CPDRigid cpd;
		cpd.setInputData(&_model, &_data);
		cpd.apply();
	}

	template <typename T, int D>
	void Registrator<T, D>::affine()
	{
		CPDAffine cpd;
		cpd.setInputData(&_model, &_data);
		cpd.apply();
	}

	template <typename T, int D>
	void Registrator<T, D>::nonrigid()
	{
		CPDNRigid cpd;
		cpd.setInputData(&_model, &_data);
		cpd.apply();
	}
}

#endif
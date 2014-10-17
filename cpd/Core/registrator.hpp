#ifndef REGISTRATOR_HPP
#define REGISTRATOR_HPP

#include "Base/data.hpp"

namespace cpd
{
	template <typename T, int D>
	class Registrator
	{
	public:
		Registrator();
		virtual ~Registrator();

		void setInputData(const TMatrixD& model, const TMatrixD& data);
		void setVision(bool vision);

		inline RegType getRegistrationType(){ return _type; }
		inline TMatrixD& getModel(){ return _model; }
		inline TMatrixD& getData(){ return _data; }

		virtual void run() = 0;

	/*private:
		void rigid();
		void affine();
		void nonrigid();*/

	protected:
		TMatrixD			_model;
		TMatrixD			_data;
		RegType				_type;
		bool				_vision;
	};
}

namespace cpd
{
	template <typename T, int D>
	Registrator<T, D>::Registrator()
		: _type(UNDEFINED), _vision(false)
	{

	}

	template <typename T, int D>
	Registrator<T, D>::~Registrator()
	{

	}

	template <typename T, int D>
	void Registrator<T, D>::setInputData(const TMatrixD& model, const TMatrixD& data)
	{
		_model = model;
		_data = data;
	}

	template <typename T, int D>
	void Registrator<T, D>::setVision(bool vision)
	{
		_vision = vision;
	}

	/*template <typename T, int D>
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
	}*/

	//template <typename T, int D>
	//void Registrator<T, D>::rigid()
	//{
	//	CPDRigid<T, D> cpd;
	//	cpd.setInputData(&_model, &_data);
	//	cpd.apply();
	//}

	//template <typename T, int D>
	//void Registrator<T, D>::affine()
	//{
	//	/*CPDAffine<T, D> cpd;
	//	cpd.setInputData(&_model, &_data);
	//	cpd.apply();*/
	//}

	//template <typename T, int D>
	//void Registrator<T, D>::nonrigid()
	//{
	//	/*CPDNRigid<T, D> cpd;
	//	cpd.setInputData(&_model, &_data);
	//	cpd.apply();*/
	//}
}

#endif
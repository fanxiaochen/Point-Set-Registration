#ifndef REGISTRATOR_HPP
#define REGISTRATOR_HPP

#include "base/data.hpp"

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

	protected:
		TMatrixD			_model;
		TMatrixD			_data;
		size_t				_M;
		size_t				_N;
		RegType				_type;
		bool				_vision;
	};
}

namespace cpd
{
	template <typename T, int D>
	Registrator<T, D>::Registrator()
		: _type(UNDEFINED), _vision(false), _M(0), _N(0)
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

		// determine data number
		_M = _model.rows();
		_N = _data.rows();
	}

	template <typename T, int D>
	void Registrator<T, D>::setVision(bool vision)
	{
		_vision = vision;
	}
}

#endif
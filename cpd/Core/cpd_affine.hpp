#ifndef CPD_AFFINE_HPP
#define CPD_AFFINE_HPP

#include "Core/cpd_base.hpp"

namespace cpd
{
	template <class T>
	class CPDAffine: public CPDBase<T>
	{
	public:
		CPDAffine();
		virtual ~CPDAffine();

		void intialization();
		void em();
		void align();

	private:

	};
}

namespace cpd
{
	template <class T>
	CPDAffine<T>::CPDAffine(){}

	template <class T>
	CPDAffine<T>::~CPDAffine(){}

	template<class T>
	void CPDAffine<T>::intialization()
	{

	}

	template<class T>
	void CPDAffine<T>::em()
	{

	}

	template<class T>
	void CPDAffine<T>::align()
	{

	}
}
#endif
#ifndef CPD_RIGID_HPP
#define CPD_RIGID_HPP

#include "Core/cpd_base.hpp"

namespace cpd
{
	template <class T>
	class CPDRigid: public CPDBase<T>
	{
	public:
		CPDRigid();
		virtual ~CPDRigid();
		void apply();

	private:
		void intialization();
		void em();
		void align();

	private:

	};
}

namespace cpd
{
	template <class T>
	CPDRigid<T>::CPDRigid(){}

	template <class T>
	CPDRigid<T>::~CPDRigid(){}

	template<class T>
	void CPDRigid<T>::apply()
	{
		std::cout << sizeof(T) << std::endl;
	}

	template<class T>
	void CPDRigid<T>::intialization()
	{

	}

	template<class T>
	void CPDRigid<T>::em()
	{

	}

	template<class T>
	void CPDRigid<T>::align()
	{

	}
}
#endif
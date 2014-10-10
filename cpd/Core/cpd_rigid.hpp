#ifndef CPD_RIGID_H
#define CPD_RIGID_H

namespace cpd
{
	template <class T>
	class CPDRigid: public CPDBase
	{
	public:
		CPDRigid();
		virtual ~CPDRigid();
		//void apply();

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

	/*template<class T>
	void CPDRigid<T>::apply()
	{

	}*/

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
#ifndef CPD_NRIGID_H
#define CPD_NRIGID_H

namespace cpd
{
	template <class T>
	class CPDNRigid: public CPDBase
	{
	public:
		CPDNRigid();
		virtual ~CPDNRigid();

		void intialization();
		void em();
		void align();

	private:

	};
}

namespace cpd
{
	template <class T>
	CPDNRigid<T>::CPDNRigid(){}

	template <class T>
	CPDNRigid<T>::~CPDNRigid(){}

	template<class T>
	void CPDNRigid<T>::intialization()
	{

	}

	template<class T>
	void CPDNRigid<T>::em()
	{

	}

	template<class T>
	void CPDNRigid<T>::align()
	{

	}
}

#endif
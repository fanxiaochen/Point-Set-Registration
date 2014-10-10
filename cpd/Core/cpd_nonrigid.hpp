#ifndef CPD_NRIGID_HPP
#define CPD_NRIGID_HPP

namespace cpd
{
	template <class T>
	class CPDNRigid: public CPDBase<T>
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
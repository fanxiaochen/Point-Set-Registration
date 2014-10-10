#ifndef CPD_AFFINE_H
#define CPD_AFFINE_H

namespace cpd
{
	template <class T>
	class CPDAffine: public CPDBase
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
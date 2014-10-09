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
#endif
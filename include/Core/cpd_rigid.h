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

		void intialization();
		void em();
		void align();

	private:

	};
}
#endif
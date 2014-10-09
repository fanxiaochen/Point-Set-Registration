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
#endif
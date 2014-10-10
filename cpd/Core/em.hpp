#ifndef EM_H
#define EM_H

#include "Base/matrix.h"

namespace cpd
{
	class EM
	{
	public:
		EM();
		~EM();
		
		void initialization();
		void expectation();
		void maximization();
		bool convergence();

	private:

	};
}

#endif
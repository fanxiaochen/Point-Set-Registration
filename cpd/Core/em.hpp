#ifndef EM_HPP
#define EM_HPP

#include "base/matrix.hpp"

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
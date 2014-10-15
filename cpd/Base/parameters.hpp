#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "Base/matrix.hpp"

namespace cpd
{
	template <typename T, int D>
	struct RigidParas  
	{
		TMatrix _R;
		TVector _t;
		T _s;
		T _sigma2;
	};

	template <typename T, int D>
	struct AffineParas  
	{
		TMatrix _B;
		TVector _t;
		T _sigma2;
	};
}

#endif
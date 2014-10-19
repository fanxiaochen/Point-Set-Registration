#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "base/matrix.hpp"

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

	template <typename T, int D>
	struct NRigidParas
	{
		TMatrix _W;
		T _sigma2;
		T _lamda;
		T _beta;
	};
}

#endif
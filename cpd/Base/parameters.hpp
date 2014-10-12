#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "Base/matrix.hpp"

namespace cpd
{
	template <typename T, int D>
	struct RigidParas  
	{
		Matrix _R;
		Vector _t;
		T _s;
		T _squared_sigma;
	};

	template <typename T, int D>
	struct AffineParas  
	{
		Matrix _B;
		Vector _t;
		T _squared_sigma;
	};
}

#endif
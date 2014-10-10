#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "Base/matrix.hpp"

namespace cpd
{
	struct RigidParas  
	{
		Matrix _R;
		Vector _t;
		value_type _s;
		value_type _squared_sigma;
	};

	struct AffineParas  
	{
		Matrix _B;
		Vector _t;
		value_type _squared_sigma;
	};
}

#endif
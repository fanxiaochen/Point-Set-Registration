#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

namespace cpd
{
	template <class T1, class T2>
	struct RigidParas  
	{
		T1 _R;
		T2 _t;
		value_type _s;
		value_type _squared_sigma;
	};

	template <class T1, class T2>
	struct AffineParas  
	{
		T1 _B;
		T2 _t;
		value_type _squared_sigma;
	};

}

#endif
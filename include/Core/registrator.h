#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include "Base/data.h"

namespace cpd
{
	template <class T>
	class Registrator
	{
	public:
		Registrator();
		~Registrator();

		void setInputData(const T& model, const T& data);
		void setType(RegType type);

		inline T& getModel(){return _model;}
		inline T& getData(){return _data;}

		void getCorrespondences();
		
		void run();

	private:
		void rigid();
		void affine();
		void nonrigid();

	private:
		T _model;
		T _data;
		RegType _type;
	};
}

#endif
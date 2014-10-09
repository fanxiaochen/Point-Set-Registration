#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include "Base/data.h"
#include "Core/cpd_base.h"

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

		T& getModel();
		T& getData();

		void getCorrespondences();
		
		void run();

	private:
		void intialization();
		void em();
		void align();

	private:
		CPDBase* _cpd_base;
		RegType _type;
	};
}

#endif
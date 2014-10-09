
namespace cpd
{
	template <class T>
	class Registrator
	{
	public:
		Registrator();
		~Registrator();

		void setInputData(const T& model, const T& data);

		inline T& getModel(){return _model;}
		inline T& getData(){return _data;}

		void getCorrespondences();
		
		void run();

	private:
		void intialization();
		void em();
		void align();

	private:
		T	_model;
		T	_data;
	};
}
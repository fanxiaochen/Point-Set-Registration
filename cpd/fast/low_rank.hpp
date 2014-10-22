#ifndef LOW_RANK_HPP
#define LOW_RANK_HPP

#include <Eigen/Eigenvalues>
#include "base/matrix.hpp"

namespace cpd
{
	template <typename T, int D>
	struct EigenType
	{
		//typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

		typedef typename Eigen::EigenSolver<TMatrix>::EigenvalueType EigenvalueType;

		typedef typename Eigen::EigenSolver<TMatrix>::EigenvectorsType EigenvectorsType;
	};
	

	template <typename T>
	struct EigenValue
	{
		EigenValue(){};

		EigenValue(T value, size_t idx)
		{
			_value = value;
			_idx = idx;
		}

		bool operator()(EigenValue i, EigenValue j)
		{
			return i._value > j._value;
		}

		T		_value;
		size_t	_idx;
	};



	template <typename T, int D>
	void lr_approximate(const TMatrix& G, TMatrix& Q, TMatrix& S, int K)
	{
		typename Eigen::EigenSolver<TMatrix> es(G);
		std::cout << es.eigenvalues() << std::endl;
		const typename EigenType<T, D>::EigenvalueType& eigen_values = es.eigenvalues();
		const typename EigenType<T, D>::EigenvectorsType& eigen_vectors = es.eigenvectors();	
		k_extract<T, D>(eigen_values, eigen_vectors, Q, S, K);
	}

	template <typename T, int D>
	void k_extract(const typename EigenType<T, D>::EigenvalueType& eigen_values, 
		const typename EigenType<T, D>::EigenvectorsType& eigen_vectors, 
		TMatrix& Q, TMatrix& S, int K)
	{
		size_t eigen_num = eigen_values.rows();

		typename std::vector<typename EigenValue<T> > ev;

		for (size_t i = 0; i < eigen_num; i ++)
		{
			typename std::complex<T> cv = eigen_values(i);
			typename EigenValue<T> i_ev(cv.real(), i);
			ev.push_back(i_ev);
		}

		std::sort(ev.begin(), ev.end(), EigenValue<T>());

		int gm = eigen_vectors.rows();
		Q.resize(gm, K);
		TVector s(K);
		
		for (size_t i = 0; i < K; i ++)
		{
			s(i) = ev[i]._value;
			typename MatrixType<std::complex<T>, D>::Matrix q_ci = eigen_vectors.col(ev[i]._idx);
			for (size_t j = 0, j_end = q_ci.rows(); j < j_end; j ++)
			{
				Q(j, i) = q_ci(j).real();
			}
			//Q.col(i) = eigen_vectors.col(ev[i]._idx);
		}

		S = s.asDiagonal();
		//std::cout << S << std::endl;
	}

}

#endif
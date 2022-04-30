#pragma once

#include <string>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>

namespace spring{
	typedef Eigen::Triplet<double> T;
	double energy(Eigen::VectorXd& x, std::vector<std::array<int, 2>>& edges, double k);
	void gradient(Eigen::VectorXd& x, std::vector<std::array<int, 2>>& edges, double k, Eigen::VectorXd& g);
	void hessian(Eigen::VectorXd& x, std::vector<std::array<int, 2>>& edges, double kk, Eigen::SparseMatrix<double>& H);
}
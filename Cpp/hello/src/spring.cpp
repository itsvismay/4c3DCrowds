#include "spring.h"

double spring::energy(Eigen::VectorXd& x, std::vector<std::array<int, 2>>& edges, double k){
  double E = 0;
  //E = 1/2 *K* (x - x0)
  for(int e=0; e<edges.size(); e++){
    std::array<int, 2> pair = edges[e];
    int i1 = pair[0];
    int i2 = pair[1];
    Eigen::Vector4d x1 = x.segment<4>(4*i1);
    Eigen::Vector4d x2 = x.segment<4>(4*i2);
    E += 0.5 * k * (x2-x1).transpose()*(x2- x1);
  }

  return E;
}

void spring::gradient(Eigen::VectorXd& x, std::vector<std::array<int, 2>>& edges, double k, Eigen::VectorXd& g){
  g.setZero();
  for(int e=0; e<edges.size(); e++){
    std::array<int, 2> pair = edges[e];
    int i1 = pair[0];
    int i2 = pair[1];
    Eigen::Vector4d x1 = x.segment<4>(4*i1);
    Eigen::Vector4d x2 = x.segment<4>(4*i2);
    g.segment<4>(4*i1) += k * (x1- x2);
    g.segment<4>(4*i2) -= k * (x1- x2);
  }
}

void spring::hessian(Eigen::VectorXd& x, std::vector<std::array<int, 2>>& edges, double kk, Eigen::SparseMatrix<double>& H){
  H.setZero();
  std::vector<T> trips;

  for(int e=0; e<edges.size(); e++){
    std::array<int, 2> pair = edges[e];
    int i1 = pair[0];
    int i2 = pair[1];
    Eigen::Vector4d x1 = x.segment<4>(4*i1);
    Eigen::Vector4d x2 = x.segment<4>(4*i2);

    H.coeffRef(4*i1 + 0, 4*i1 + 0) += kk;
    H.coeffRef(4*i1 + 1, 4*i1 + 1) += kk;
    H.coeffRef(4*i1 + 2, 4*i1 + 2) += kk;
    H.coeffRef(4*i1 + 3, 4*i1 + 3) += kk;

    H.coeffRef(4*i2 + 0, 4*i1 + 0) -= kk;
    H.coeffRef(4*i2 + 1, 4*i1 + 1) -= kk;
    H.coeffRef(4*i2 + 2, 4*i1 + 2) -= kk;
    H.coeffRef(4*i2 + 3, 4*i1 + 3) -= kk;

    H.coeffRef(4*i1 + 0, 4*i2 + 0) -= kk;
    H.coeffRef(4*i1 + 1, 4*i2 + 1) -= kk;
    H.coeffRef(4*i1 + 2, 4*i2 + 2) -= kk;
    H.coeffRef(4*i1 + 3, 4*i2 + 3) -= kk;

    H.coeffRef(4*i2 + 0, 4*i2 + 0) += kk;
    H.coeffRef(4*i2 + 1, 4*i2 + 1) += kk;
    H.coeffRef(4*i2 + 2, 4*i2 + 2) += kk;
    H.coeffRef(4*i2 + 3, 4*i2 + 3) += kk;



  //   for(int j =0; j<4; ++j)
  //   {
  //     for(int k =0; k<4; ++k)
  //     {
  //       H.coeffRef(4*i1 + j, 4*i1 + k) += -kk;
  //       H.coeffRef(4*i2 + j, 4*i2 + k) += -kk;
  //       H.coeffRef(4*i1 + j, 4*i2 + k) += kk;
  //       H.coeffRef(4*i2 + j, 4*i1 + k) += kk;
  //       // trips.push_back(spring::T(4*i1 + j, 4*i1 + k, -k));
  //       // trips.push_back(spring::T(4*i2 + j, 4*i2 + k, -k));
  //       // trips.push_back(spring::T(4*i1 + j, 4*i2 + k,  k));
  //       // trips.push_back(spring::T(4*i2 + j, 4*i1 + k,  k));
  //     }
  //   }
  }

  // H.setFromTriplets(trips.begin(), trips.end());
}

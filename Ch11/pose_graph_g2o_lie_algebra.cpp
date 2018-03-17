#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <sophus/se3.h>
#include <sophus/so3.h>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

Matrix6d JRInv(SE3 e)
{
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3::hat(e.so3().log()); // Logarithmic map
    J.block(0, 3, 3, 3) = SO3::hat(e.translation());
    J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3::hat(e.so3().log());

    J = J * 0.5 + Matrix6d::Identify();

    return J;
}

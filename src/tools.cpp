/*______________________________________________________________________________
# tools.cpp                                                                80->|
# This module implements the Tools class
*/

#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Code from my answer to L5.22 quiz
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity of the inputs:
  // the estimation vector size should not be zero
  assert(estimations.size() > 0);
  // the estimation vector size should equal ground truth vector size
  assert(estimations.size() == ground_truth.size());

  //accumulate squared residuals
  for(int i = 0; i < estimations.size(); i++) {
    VectorXd r = estimations[i] - ground_truth[i];
    r = r.array() * r.array();
    rmse += r;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  //recover state parameters
  float px = x_state(0), py = x_state(1), vx = x_state(2), vy = x_state(3);

  // Code from my answer to L5.18 quiz
  float eps = 1e-9;
  float px2 = px * px, py2 = py * py;
  float xySum = px2 + py2;
  float xySqrt = sqrt(xySum) + eps;
  float xyPow = pow(xySum, 1.5) + eps;
  xySum += eps;
  //check for division by zero
  if (px == 0 && py == 0)
    Hj << 0,    0,    0,   0,
         -1e+9, 1e+9, 0,   0,
          0,    0,    0,   0;
  else
    //compute the Jacobian matrix
    Hj << px / xySqrt, py / xySqrt, 0.0, 0.0,
    -py / xySum,  px / xySum, 0.0, 0.0,
    py*(vx * py - vy * px) / xyPow, px*(vy * px - vx * py) / xyPow, 
    px / xySqrt, py / xySqrt;
  return Hj;
}

/*______________________________________________________________________________
# kalman_filter.cpp                                                        80->|
# This module implements the KalmanFilter class
*/

#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

// Passing by reference
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Code from my answer to L5.12 quiz
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Code from my answer to L5.12 quiz
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  // compute new estimate
  x_ = x_ + (K * y);
  P_ = P_ - K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Update the state by using Extended Kalman Filter equations.
  // L6.8 Tips and Tricks followed by code from my answer to L5.12 quiz

  // Use h(x') to calculate y...
  float px = x_[0], py = x_[1], vx = x_[2], vy = x_[3];
  float ro = sqrt(px * px + py * py), phi, ro_dot;
  VectorXd h = VectorXd(3);
  if (ro < 1e-7) {
    h << 0.0, 0.0, 0.0;  // avoid div by zero
    }
  else {
    phi = atan2(py, px);
    ro_dot = (px * vx + py * vy) / ro;
    h << ro, phi, ro_dot;
  }
  VectorXd y = z - h;
  // After subtraction, normalize phi to range [-pi, pi]
  // atan trick: bit.ly/2o7CNwK
  y(1) = atan2(sin(y(1)), cos(y(1)));

  // Use Hj_ to calculate S, K, and P
  MatrixXd Hj_ = H_; // copy the pre-computed Jacobian matrix from H_
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_;
  MatrixXd K = P_ * Hjt * S.inverse();

  // compute new estimate
  x_ = x_ + (K * y);
  P_ = P_ - K * Hj_ * P_;
}

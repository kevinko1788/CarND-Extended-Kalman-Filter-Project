#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // estimate
  P_ = P_in; // uncertainty covariance
  F_ = F_in; // state transition matrix
  H_ = H_in; // Measurement matrix
  R_ = R_in; // Measurement covariance matrix 
  Q_ = Q_in; // Process covariance matrix
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  matrixXd Ft = F_.transpose();
  P_ = F_ * P_* Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  matrixXd y = z - H_ * x_;

  CommonUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // for radar
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double theta = atan2(py,px);
  double rho_dot = (px*vx + py*vy) / rho

  VectorXd h = VectorXd(3);

  h << rho, theta, rho_dot;

  y = z - h;

  CommonUpdate(y);
}

void KalmanFilter::CommonUpdate(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I-K* H_) * P_;
}

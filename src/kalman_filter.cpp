#include "kalman_filter.h"
#include <math.h>
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);


  //Initialize covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  //measurement matrix
  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // Initialize transition matrix
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);

//  Tools tools;
}

KalmanFilter::~KalmanFilter() {}

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(Eigen::VectorXd z, Eigen::MatrixXd R_laser) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}

VectorXd KalmanFilter::measure_f(const Eigen::VectorXd x) {
  float px = x[0];
  float py = x[1];
  float vx = x[2];
  float vy = x[3];
  pred_ = VectorXd(3);

  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  float c4 = atan(py/px);
  float c5 = px*vx + py*vy;

  pred_ << c2, c4, c5/c2;

  return pred_;

}

void KalmanFilter::UpdateEKF(Eigen::VectorXd z,  Eigen::MatrixXd R_radar) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd pred = measure_f(x_);
  MatrixXd Hj_ = tools.CalculateJacobian(x_);
  VectorXd y = z - pred;
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar ;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;

}

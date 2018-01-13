#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

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
 	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateNewEstimate(const VectorXd &y) {
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
 	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	
	UpdateNewEstimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float rho = sqrt(px*px + py*py);
	float theta = atan2(py, px);
	float ro_dot = (px*vx + py * vy) / rho;
	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, ro_dot;

	VectorXd y = z - z_pred;

	/*Normalizing Angles
	In C++, atan2() returns values between - pi and pi.When calculating phi in y = z - h(x)
	for radar measurements, the resulting angle phi in the y vector should be adjusted so
	that it is between - pi and pi.The Kalman filter is expecting small angle values
	between the range - pi and pi.HINT: when working in radians, you can add 2*pi?
	or subtract 2*pi? until the angle is within the desired range.
	*/
	y(1) = atan2(sin(y(1)), cos(y(1)));
	// This works, but not sure this is correct logic.
	//if (y(1) > M_PI)
	//	y(1) = y(1) - 2* M_PI;
	//else if (y(1) < -M_PI)
	//	y(1) =  y(1) + 2 *M_PI;

	UpdateNewEstimate(y);
}

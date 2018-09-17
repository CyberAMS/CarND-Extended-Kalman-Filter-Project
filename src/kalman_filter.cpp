#include "kalman_filter.h"

#define PI acos(-1.0)

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
	/**
	TODO:
		* predict the state
	*/
  	
	// predict state
	x_ = F_ * x_;
	
	// predict noise
	MatrixXd Ft = F_.transpose();
	P_ = (F_ * P_ * Ft) + Q_;
	
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Kalman Filter equations
	*/
	
	// calculate y
	VectorXd y = z - H_ * x_;

	// call function to update with this y value
	UpdateWithY(y);
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Extended Kalman Filter equations
	*/
	
	// define constants
	const int NUM_RADAR_MEASUREMENTS = 3;

	// get cartesian state variables
  float px;
  float py;
  float vx;
  float vy;
	x_ >> px, py, vx, vy;
	
	// calculate polar state variables
  float rho = sqrt((px * px) + (py * py));
  float theta = atan2(py, px);
  float rho_dot = ((px * vx) + (py * vy)) / rho;
	
	// calculate h
  VectorXd h = VectorXd(NUM_RADAR_MEASUREMENTS);
  h << rho, theta, rho_dot;

	// calculate y
	VectorXd y = z - h;
  while (y(1) > PI || y(1) < -PI ) {
    if (y(1) > PI) {
      y(1) -= PI;
    } else {
      y(1) += M_PI;
    }
	}
	
	// call function to update with this y value
	UpdateWithY(y);
	
}

void KalmanFilter::UpdateWithY(const VectorXd &y){
	
	// define constants
	const int NUM_STATES = 4;

	// calculate identity matrix
	MatrixXd I = MatrixXd::Identity(NUM_STATES, NUM_STATES);
	
	// calculate matrices
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;

	// new state and noise
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
	
}
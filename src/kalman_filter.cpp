#include "kalman_filter.h"

#define PI acos(-1.0)

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
	
	// define constants
	const bool bDISPLAY = true;
	
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
  
	// display message if required
	if bDISPLAY {
		cout << "KALMAN: Predict - Start" << endl;
		cout << "  Model F: " << F_ << endl;
		cout << "  State x (px, py, vx, vy): " << x_ << endl;
		cout << "  Error noise P: " << P_ << endl;
		cout << "  Process noise Q: " << Q_ << endl;
	}
	
	// predict state
	x_ = F_ * x_;
	
	// predict noise
	MatrixXd Ft = F_.transpose();
	P_ = (F_ * P_ * Ft) + Q_;
	
	// display message if required
	if bDISPLAY {
		cout << "  Inverse model Ft: " << Ft << endl;
		cout << "  New state x (px, py, vx, vy): " << x_ << endl;
		cout << "  New error noise P: " << P_ << endl;
		cout << "--- KALMAN: Predict - End" << endl;
	}
	
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Kalman Filter equations
	*/
	
	// display message if required
	if bDISPLAY {
		cout << "KALMAN: Update - Start" << endl;
		cout << "  Measurement z: " << z << endl;
		cout << "  Measurement matrix H: " << H_ << endl;
		cout << "  State x (px, py, vx, vy): " << x_ << endl;
	}
	
	// calculate y
	VectorXd y = z - H_ * x_;

	// call function to update with this y value
	UpdateWithY(y);
	
	// display message if required
	if bDISPLAY {
		cout << "  Measurement post fit y: " << y << endl;
		cout << "--- KALMAN: Update - End" << endl;
	}
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Extended Kalman Filter equations
	*/
	
	// define constants
	const int NUM_RADAR_MEASUREMENTS = 3;
	const int ZERO_DETECTION = 0.0001;

	// display message if required
	if bDISPLAY {
		cout << "KALMAN: UpdateEKF - Start" << endl;
		cout << "  Measurement z: " << z << endl;
		cout << "  State x (px, py, vx, vy): " << x_ << endl;
	}

	// get cartesian state variables
  float px;
  float py;
  float vx;
  float vy;
	x_ >> px, py, vx, vy;
	
	// calculate polar state variables
  float rho = sqrt((px * px) + (py * py));
  if (fabs(rho) < ZERO_DETECTION) {
		rho = ((rho > 0) - (rho < 0)) * ZERO_DETECTION; // avoid value close to zero - retain sign
	}
  float theta = atan2(py, px);
  float rho_dot = ((px * vx) + (py * vy)) / rho;
	
	// calculate h
  VectorXd h = VectorXd(NUM_RADAR_MEASUREMENTS);
  hx << rho, theta, rho_dot;

	// calculate y
	VectorXd y = z - hx;
  while (y(1) > PI || y(1) < -PI ) {
    if (y(1) > PI) {
      y(1) -= PI;
    } else {
      y(1) += M_PI;
    }
	}
	
	// call function to update with this y value
	UpdateWithY(y);
	
	// display message if required
	if bDISPLAY {
		cout << "  Measurement function hx (roh, theta, rho_dot): " << hx << endl;
		cout << "  Measurement post fit y: " << y << endl;
		cout << "--- KALMAN: UpdateEKF - End" << endl;
	}
	
}

void KalmanFilter::UpdateWithY(const VectorXd &y){
	
	// define constants
	const int NUM_STATES = 4;

	// display message if required
	if bDISPLAY {
		cout << "KALMAN: UpdateWithY - Start" << endl;
		cout << "  Measurement post fit y: " << y << endl;
		cout << "  State x (px, py, vx, vy): " << x_ << endl;
		cout << "  Error noise P: " << P_ << endl;
		cout << "  Measurement matrix H: " << H_ << endl;
	}

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
	
	// display message if required
	if bDISPLAY {
		cout << "  Identity matrix I: " << I << endl;
		cout << "  Measurement matrix transposed Ht: " << Ht << endl;
		cout << "  Innovation covariance matrix S: " << S << endl;
		cout << "  Innovation covariance matrix inversed Si: " << Si << endl;
		cout << "  Optimal KALMAN gain matrix K: " << K << endl;
		cout << "  New state x (px, py, vx, vy): " << x_ << endl;
		cout << "  New error noise P: " << P_ << endl;
		cout << "--- KALMAN: UpdateWithY - End" << endl;
	}

}
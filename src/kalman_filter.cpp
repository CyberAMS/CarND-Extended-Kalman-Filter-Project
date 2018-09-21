#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "tools.h"
#include "measurement_package.h"

#define PI acos(-1.0)

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
	
	// initialize matrices
	y_laser = VectorXd(NUM_LASER_MEASUREMENTS);
	y_laser << 1, 1;
	y_radar = VectorXd(NUM_RADAR_MEASUREMENTS);	
	y_radar << 1, 1, 1;
  hx = VectorXd(NUM_RADAR_MEASUREMENTS);
  hx << 1, 1, 1;
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
	if (bDISPLAY) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "KALMAN: Predict - Start" << endl;
		cout << "  Model F: " << endl << F_ << endl;
		cout << "  State x (px, py, vx, vy): " << endl << x_ << endl;
		cout << "  Error noise P: " << endl << P_ << endl;
		cout << "  Process noise Q: " << endl << Q_ << endl;
	}
	
	// predict state
	x_ = F_ * x_;
	
	// predict noise
	Ft = F_.transpose();
	P_ = (F_ * P_ * Ft) + Q_;
	
	// display message if required
	if (bDISPLAY) {
		cout << "  Inverse model Ft: " << endl << Ft << endl;
		cout << "  New state x (px, py, vx, vy): " << endl << x_ << endl;
		cout << "  New error noise P: " << endl << P_ << endl;
		cout << "--- KALMAN: Predict - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
	}
	
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Kalman Filter equations
	*/
	
	// display message if required
	if (bDISPLAY) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "KALMAN: Update - Start" << endl;
		cout << "  Measurement z: " << endl << z << endl;
		cout << "  Measurement matrix H: " << endl << H_ << endl;
		cout << "  State x (px, py, vx, vy): " << endl << x_ << endl;
	}
	
	// calculate y
	y_laser = z - H_ * x_;

	// call function to update with this y_laser value
	UpdateWithY(y_laser);
	
	// display message if required
	if (bDISPLAY) {
		cout << "  Measurement post fit y: " << endl << y_laser << endl;
		cout << "--- KALMAN: Update - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
	}
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Extended Kalman Filter equations
	*/
	
	// display message if required
	if (bDISPLAY) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "KALMAN: UpdateEKF - Start" << endl;
		cout << "  Measurement z: " << endl << z << endl;
		cout << "  State x (px, py, vx, vy): " << endl << x_ << endl;
	}

	// get cartesian state variables
	px = x_(0);
	py = x_(1);
	vx = x_(2);
	vy = x_(3);
	
	// calculate polar state variables
  rho = sqrt((px * px) + (py * py));
  if (fabs(rho) < ZERO_DETECTION) {
		rho = ((rho > 0) - (rho < 0)) * ZERO_DETECTION; // avoid value close to zero - retain sign
	}
  theta = atan2(py, px);
  rho_dot = ((px * vx) + (py * vy)) / rho;
	
	// calculate h
  hx(0) = rho;
	hx(1) = theta;
	hx(2) = rho_dot;

	// calculate y
	y_radar = z - hx;
  while (y_radar(1) > PI || y_radar(1) < -PI ) {
    if (y_radar(1) > PI) {
      y_radar(1) -= PI;
    } else {
      y_radar(1) += PI;
    }
	}
	
	// call function to update with this y_radar value
	UpdateWithY(y_radar);
	
	// display message if required
	if (bDISPLAY) {
		cout << "  Measurement function hx (roh, theta, rho_dot): " << hx << endl;
		cout << "  Measurement post fit y: " << y_radar << endl;
		cout << "--- KALMAN: UpdateEKF - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
	}
	
}

void KalmanFilter::UpdateWithY(const VectorXd &y){
	
	// display message if required
	if (bDISPLAY) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "KALMAN: UpdateWithY - Start" << endl;
		cout << "  Measurement post fit y: " << endl << y << endl;
		cout << "  State x (px, py, vx, vy): " << endl << x_ << endl;
		cout << "  Error noise P: " << endl << P_ << endl;
		cout << "  Measurement matrix H: " << endl << H_ << endl;
	}

	// calculate identity matrix
	I = MatrixXd::Identity(NUM_STATES, NUM_STATES);
	
	// calculate matrices
	Ht = H_.transpose();
	S = H_ * P_ * Ht + R_;
	Si = S.inverse();
	K =  P_ * Ht * Si;

	// new state and noise
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
	
	// display message if required
	if (bDISPLAY) {
		cout << "  Identity matrix I: " << endl << I << endl;
		cout << "  Measurement matrix transposed Ht: " << endl << Ht << endl;
		cout << "  Innovation covariance matrix S: " << endl << S << endl;
		cout << "  Innovation covariance matrix inversed Si: " << endl << Si << endl;
		cout << "  Optimal KALMAN gain matrix K: " << endl << K << endl;
		cout << "  New state x (px, py, vx, vy): " << endl << x_ << endl;
		cout << "  New error noise P: " << endl << P_ << endl;
		cout << "--- KALMAN: UpdateWithY - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
	}

}
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class KalmanFilter {

public:

	// state vector
	VectorXd x_;

	// state covariance matrix
	MatrixXd P_;

	// state transition matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;

	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Init Initializes Kalman filter
	 * @param x_in Initial state
	 * @param P_in Initial state covariance
	 * @param F_in Transition matrix
	 * @param H_in Measurement matrix
	 * @param R_in Measurement covariance matrix
	 * @param Q_in Process covariance matrix
	 */
	void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
	    Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model
	 * @param delta_T Time between k and k+1 in s
	 */
	 // => Not sure why template comment lists a parameter here!
	void Predict();

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void Update(const Eigen::VectorXd &z);

	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void UpdateEKF(const Eigen::VectorXd &z);

private:

	// define matrics
	MatrixXd Ft;
	VectorXd y_laser;
	VectorXd y_radar;
	VectorXd hx;
	MatrixXd I;
	MatrixXd Ht;
	MatrixXd S;
	MatrixXd Si;
	MatrixXd K;

	// define polar coordinates
	double rho;
	double phi;
	double rho_dot;

	// define cartesian coordinates
	double px;
	double py;
	double vx;
	double vy;

	// boolean for display debug messages
	const bool bDISPLAY = true;

	// define constants
	const int NUM_LASER_MEASUREMENTS = 2;
	const int NUM_RADAR_MEASUREMENTS = 3;
	const int NUM_STATES = 4;
	const double ZERO_DETECTION = 0.0001;

	/**
	 * Updates the state using y
	 * @param y The measurement post fit
	 */
	void UpdateWithY(const VectorXd &y);

};

#endif /* KALMAN_FILTER_H_ */
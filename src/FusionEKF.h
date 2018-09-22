#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "tools.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class FusionEKF {

public:

	/**
	* Constructor.
	*/
	FusionEKF();

	/**
	* Destructor.
	*/
	virtual ~FusionEKF();

	/**
	* Run the whole flow of the Kalman Filter from here.
	*/
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	/**
	* Kalman Filter update and prediction math lives in here.
	*/
	KalmanFilter ekf_;

private:

	// check whether the tracking toolbox was initialized or not (first measurement)
	bool is_initialized_;

	// previous timestamp
	long long previous_timestamp_;

	// tool object used to compute Jacobian and RMSE
	Tools tools;

	// define matrices
	MatrixXd R_laser_;
	MatrixXd R_radar_;
	MatrixXd H_laser_;
	MatrixXd Hj_;
	VectorXd x_init;
	MatrixXd P_init;
	MatrixXd F_init;
	MatrixXd Q_init;

	// define polar coordinates
	double rho;
	double theta;
	double rho_dot;

	// define cartesian coordinates
	double px;
	double py;
	double vx;
	double vy;

	// define current time stamp and delta time steps
	long long current_timestamp;
	double dt;
	double dt_2;
	double dt_3;
	double dt_4;

	// define boolean for display debug messages
	const bool bDISPLAY = true;

	// define constants for variable dimensions
	const int NUM_LASER_MEASUREMENTS = 2;
	const int NUM_RADAR_MEASUREMENTS = 3;
	const int NUM_STATES = 4;
	const int NUM_OBSERVABLE_STATES = 2;
	const int ZERO_DETECTION = 0.0001;
	const double NOISE_AX = 9.0;
	const double NOISE_AY = 9.0;

};

#endif /* FusionEKF_H_ */
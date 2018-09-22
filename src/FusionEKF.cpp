#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "kalman_filter.h"
#include "tools.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(NUM_LASER_MEASUREMENTS, NUM_LASER_MEASUREMENTS);
	R_radar_ = MatrixXd(NUM_RADAR_MEASUREMENTS, NUM_RADAR_MEASUREMENTS);
	H_laser_ = MatrixXd(NUM_OBSERVABLE_STATES, NUM_STATES);
	Hj_ = MatrixXd(NUM_RADAR_MEASUREMENTS, NUM_STATES);

	// measurement covariance matrix - laser (actual values)
	R_laser_ << 0.0225,      0,
	                 0, 0.0225;

	// measurement covariance matrix - radar (actual values)
	R_radar_ << 0.09,      0,    0,
	               0, 0.0009,    0,
	               0,      0, 0.09;

	/**
	TODO:
		* Finish initializing the FusionEKF.
		* Set the process and measurement noises
	*/

	// measurement matrix (actual values)
	H_laser_ << 1, 0, 0, 0,
	            0, 1, 0, 0;

	// Jacobian for measurement noise template (template)
	Hj_ << 1, 1, 0, 0,
	       1, 1, 0, 0,
	       1, 1, 1, 1;

	// state vector (template)
	x_init = VectorXd(NUM_STATES);
	x_init << 1, 1, 1, 1;

	// initial error covariance matrix (actual values)
	P_init = MatrixXd(NUM_STATES, NUM_STATES);
	P_init << 1, 0,    0,    0, // measurement for position available at the beginning
	          0, 1,    0,    0, // measurement for position available at the beginning
	          0, 0, 1000,    0, // very uncertain velocity at the beginning
	          0, 0,    0, 1000; // very uncertain velocity at the beginning

	// state-transition matrix (template)
	F_init = MatrixXd(NUM_STATES, NUM_STATES);
	F_init << 1, 0, 1, 0,
	          0, 1, 0, 1,
	          0, 0, 1, 0,
	          0, 0, 0, 1;

	// process noise covariance matrix (template)
	Q_init = MatrixXd(NUM_STATES, NUM_STATES);
	Q_init << 1, 0, 1, 0,
	          0, 1, 0, 1,
	          1, 0, 1, 0,
	          0, 1, 0, 1;

	// initialize KALMAN filter object
	ekf_.Init(x_init, P_init, F_init, H_laser_, R_laser_, Q_init);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	// display message if required
	if (bDISPLAY) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "EKF: ProcessMeasurement - Start" << endl;
		cout << "  Time stamp: " << measurement_pack.timestamp_ << endl;
		cout << "  Sensor type: " << measurement_pack.sensor_type_ << endl;
		cout << "  Raw measurements: " << endl << measurement_pack.raw_measurements_ << endl;
		cout << "  Inititialized: " << is_initialized_ << endl;
	}

	/*****************************************************************************
	 * Initialization
	 ****************************************************************************/
	if (!is_initialized_) {
		
		/**
		TODO:
			* Initialize the state ekf_.x_ with the first measurement.
			* Create the covariance matrix.
			* Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		
		// first measurement => not necessary
		// ekf_.x_ = VectorXd(4);
		// ekf_.x_ << 1, 1, 1, 1;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/

			// collect initial state values
			rho = measurement_pack.raw_measurements_[0];
			theta = measurement_pack.raw_measurements_[1];
			rho_dot = measurement_pack.raw_measurements_[2];

			// coordinate convertion from polar to cartesian
			px = rho * cos(theta);
			if (fabs(px) < ZERO_DETECTION) {
				px = ((px > 0) - (px < 0)) * ZERO_DETECTION; // avoid value close to zero - retain sign
			}
			py = rho * sin(theta);
			if (fabs(py) < ZERO_DETECTION) {
				py = ((py > 0) - (py < 0)) * ZERO_DETECTION; // avoid value close to zero - retain sign
			}
			vx = rho_dot * cos(theta);
			vy = rho_dot * sin(theta);
			
		} else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

			/**
			Initialize state.
			*/

			// collect initial state values
			px = measurement_pack.raw_measurements_[0];
			py = measurement_pack.raw_measurements_[1];
			vx = 0;
			vy = 0;

		} else {

			/**
			No valid measurements.
			*/

			// collect initial state values
			px = 0;
			py = 0;
			vx = 0;
			vy = 0;

		}

		// assign values to initial state vector
		ekf_.x_(0) = px;
		ekf_.x_(1) = py;
		ekf_.x_(2) = vx;
		ekf_.x_(3) = vy;

		// save first time stamp
		previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;

		if (bDISPLAY) {
			cout << "  Inititialized: " << is_initialized_ << endl;
			cout << "  EKF state x: " << endl << ekf_.x_ << endl;
			cout << "  EKF error noise P: " << endl << ekf_.P_ << endl;
			cout << "--- EKF: ProcessMeasurement - End" << endl;
			cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		}

		return;

	}

	/*****************************************************************************
	 * Prediction
	 ****************************************************************************/

	// get current time stamp	
	current_timestamp = measurement_pack.timestamp_;

		/**
	TODO:
		* Update the state transition matrix F according to the new elapsed time.
			- Time is measured in seconds.
		* Update the process noise covariance matrix.
		* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/

	dt = (current_timestamp - previous_timestamp_) / 1000.0 / 1000.0;
	dt_2 = dt * dt;
	dt_3 = dt_2 * dt;
	dt_4 = dt_3 * dt;

	// update the state transition matrix
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	// update the process noise covariance matrix
	ekf_.Q_(0, 0) = (dt_4 / 4) * NOISE_AX;
	ekf_.Q_(0, 2) = (dt_3 / 2) * NOISE_AX;
	ekf_.Q_(1, 1) = (dt_4 / 4) * NOISE_AY;
	ekf_.Q_(1, 3) = (dt_3 / 2) * NOISE_AY;
	ekf_.Q_(2, 0) = (dt_3 / 2) * NOISE_AX;
	ekf_.Q_(2, 2) = dt_2 * NOISE_AX;
	ekf_.Q_(3, 1) = (dt_3 / 2) * NOISE_AY;
	ekf_.Q_(3, 3) = dt_2 * NOISE_AY;

	// call prediction step
	ekf_.Predict();

	/*****************************************************************************
	 * Update
	 ****************************************************************************/

	/**
	TODO:
		* Use the sensor type to perform the update step.
		* Update the state and covariance matrices.
	*/

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates

		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

	} else {
		// Laser updates

		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);

	}

	// save current time stamp
	previous_timestamp_ = current_timestamp;

	// print the output
	if (bDISPLAY) {
		cout << "  Delta time: " << dt << endl;
		cout << "  EKF model F: " << endl << ekf_.F_ << endl;
		cout << "  EKF process noise Q: " << endl << ekf_.Q_ << endl;
		cout << "  EKF state x: " << endl << ekf_.x_ << endl;
		cout << "  EKF error noise P: " << endl << ekf_.P_ << endl;
		cout << "--- EKF: ProcessMeasurement - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
	}

}
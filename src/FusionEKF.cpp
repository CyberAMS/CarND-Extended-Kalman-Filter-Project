#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	
	// define constants
	const int NUM_LASER_MEASUREMENTS = 2;
	const int NUM_RADAR_MEASUREMENTS = 3;
	const int NUM_STATES = 4;
	const int NUM_OBSERVABLE_STATES = 2;

	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(NUM_LASER_MEASUREMENTS, NUM_LASER_MEASUREMENTS);
	R_radar_ = MatrixXd(NUM_RADAR_MEASUREMENTS, NUM_RADAR_MEASUREMENTS);
	H_laser_ = MatrixXd(NUM_OBSERVABLE_STATES, NUM_STATES);
	Hj_ = MatrixXd(NUM_RADAR_MEASUREMENTS, NUM_STATES);

	// measurement covariance matrix - laser
	R_laser_ << 0.0225,      0,
                   0, 0.0225;

	// measurement covariance matrix - radar
	R_radar_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;

	/**
	TODO:
		* Finish initializing the FusionEKF.
		* Set the process and measurement noises
	*/
	
	// measurement matrix
	H_laser_ << 1, 0, 0, 0,
						  0, 1, 0, 0;
			   
	// Jacobian for measurement noise
	Hj_ << 1, 1, 0, 0,
	       1, 1, 0, 0,
		     1, 1, 1, 1;
	
	// initialize KALMAN filter object
	x_init = VectorXd(NUM_STATES);
	x_init << 1, 1, 1, 1;
	P_init = MatrixXd(NUM_STATES, NUM_STATES);
	P_init << 1, 0,    0,    0, // measurement for position available at the beginning
						0, 1,    0,    0, // measurement for position available at the beginning
						0, 0, 1000,    0, // very uncertain velocity at the beginning
						0, 0,    0, 1000; // very uncertain velocity at the beginning
	F_init = MatrixXd(NUM_STATES, NUM_STATES);
	F_init << 1, 0, 1, 0,
						0, 1, 0, 1,
						0, 0, 1, 0,
						0, 0, 0, 1;
	Q_init = MatrixXd(NUM_STATES, NUM_STATES);
	Q_init << 1, 0, 1, 0,
						0, 1, 0, 1,
						1, 0, 1, 0,
						0, 1, 0, 1;
	ekf_.Init(x_init, P_init, F_init, H_laser_, R_laser_, Q_init);
	
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	
	// define constants
	const int ZERO_DETECTION = 0.0001;
  const float NOISE_AX = 9.0;
	const float NOISE_AY = 9.0;
	
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
		
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
		
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			
			// collect initial state values
      float roh;
      float theta;
      float roh_dot;
      meas_package.raw_measurements_ >> roh, theta, roh_dot;
			
  	  // coordinate convertion from polar to cartesian
  	  float px = rho * cos(phi);
      if (px < ZERO_DETECTION) {
        x = ZERO_DETECTION; // avoid value close to zero
      }
  	  float py = rho * sin(phi);
      if (py < ZERO_DETECTION) {
        py = ZERO_DETECTION; // avoid value close to zero
      }
  	  float vx = rho_dot * cos(phi);
			float vy = rho_dot * sin(phi);
			
			// assign values to initial state vector
			ekf_.x_ << px, py, vx, vy;	  
	  
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			
      /**
      Initialize state.
      */
	  
			// collect initial state values
			float px;
			float py;
			meas_package.raw_measurements_ >> px, py;
			float vx = 0;
			float vy = 0;
			
			// assign values to initial state vector
			ekf_.x_ << px, py, vx, vy;
	  
    }
		
		// done initializing, no need to predict or update
    is_initialized_ = true;
		
    return;
		
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
	
	// get current time stamp	
	long long current_timestamp = measurement_pack.timestamp_;
	
  /**
  TODO:
    * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
    * Update the process noise covariance matrix.
    * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  */
	
  float dt = (current_timestamp - previous_timestamp_) / 1000.0 / 1000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
	
	// update the state transition matrix
	ekf_.F_(1, 3) = dt;
	ekf_.F_(2, 4) = dt;

	// update the process noise covariance matrix
	ekf_.Q_(1, 1) = (dt_4 / 4) * NOISE_AX;
	ekf_.Q_(1, 3) = (dt_3 / 2) * NOISE_AX;
	ekf_.Q_(2, 2) = (dt_4 / 4) * NOISE_AY;
	ekf_.Q_(2, 4) = (dt_3 / 2) * NOISE_AY;
	ekf_.Q_(3, 1) = (dt_3 / 2) * NOISE_AX;
	ekf_.Q_(3, 3) = dt_2 * NOISE_AX;
	ekf_.Q_(4, 2) = (dt_3 / 2) * NOISE_AY;
	ekf_.Q_(4, 4) = dt_2 * NOISE_AY;
	
	// call prediction step
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
  TODO:
    * Use the sensor type to perform the update step.
    * Update the state and covariance matrices.
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	
	ekf_.UpdateEKF(meas_package.raw_measurements_);
	
  } else {
    // Laser updates
	
	ekf_.Update(meas_package.raw_measurements_);
	
  }
	
	// save current time stamp
	previous_timestamp_ = current_timestamp;
	
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
	
}
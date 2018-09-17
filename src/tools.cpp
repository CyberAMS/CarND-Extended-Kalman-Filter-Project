#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	/**
	TODO:
		* Calculate the RMSE here.
	*/
	
	// define constants
	const int NUM_STATES = 4;
	
	// initialize output
	VectorXd rmse(NUM_STATES);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
		
		cout << "Invalid estimation or ground_truth data" << endl;
		
		return rmse;
		
	}

	// accumulate squared residuals
	for(unsigned int i = 0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		// coefficient-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
		
	}

	// calculate the mean
	rmse = rmse / estimations.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

	// return the result
	return rmse;
	
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	TODO:
		* Calculate a Jacobian here.
	*/

	// define constants
	const int NUM_MEASUREMENTS = 3;
	const int NUM_STATES = 4;
	const int ZERO_DETECTION = 0.0001;
	
	// initialize output
	MatrixXd Hj(NUM_MEASUREMENTS, NUM_STATES);
	Hj << 0, 0, 0, 0,
	      0, 0, 0, 0,
		    0, 0, 0, 0;
	
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	float c1 = ((px * px) + (py * py));
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	//check division by zero (it's sufficient to check c1 as c2 and c3 only depend on c1 being zero)
	if(fabs(c1) < ZERO_DETECTION){
		
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		
		return Hj;
		
	}

	//compute the Jacobian matrix
	Hj <<  (px / c2), (py / c2), 0, 0,
		    -(py / c1), (px / c1), 0, 0,
		    ((py * ((vx * py) - (vy * px))) / c3), ((px * ((px * vy) - (py * vx))) / c3), (px / c2), (py / c2);

  // return the result
	return Hj;
	
}
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
	
	// initialize matrices
	rmse = VectorXd(NUM_STATES);
	rmse << 1, 1, 1, 1;
	Hj = MatrixXd(NUM_MEASUREMENTS, NUM_STATES);
	Hj << 1, 1, 0, 0,
	      1, 1, 0, 0,
		    1, 1, 1, 1;
	
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	/**
	TODO:
		* Calculate the RMSE here.
	*/
	
	// initialize output
	rmse(0) = 0;
	rmse(1) = 0;
	rmse(2) = 0;
	rmse(3) = 0;
	
	// display message if required	
	if (bDISPLAY) {
		cout << "Tools: CalculateRMSE - Start" << endl;
		// cout << "  Estimations: " << estimations << endl; // produces error message
		// cout << "  Ground truth: " << ground_truth << endl; // produces error message
	}

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
		
		cout << "Invalid estimation or ground_truth data" << endl;
		
		return rmse;
		
	}

	// accumulate squared residuals
	for(vCount = 0; vCount < estimations.size(); ++vCount){

		residual = estimations[vCount] - ground_truth[vCount];

		// coefficient-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
		
	}

	// calculate the mean
	rmse = rmse / estimations.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

	// display message if required	
	if (bDISPLAY) {
		cout << "  RMSE: " << rmse << endl;
		cout << "--- Tools: CalculateRMSE - End" << endl;
	}
	
	// return the result
	return rmse;
	
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	TODO:
		* Calculate a Jacobian here.
	*/

	// display message if required	
	if (bDISPLAY) {
		cout << "Tools: CalculateJacobian - Start" << endl;
		cout << "  State x (px, py, vx, vy): " << x_state << endl;
	}

	// recover state parameters
	px = x_state(0);
	py = x_state(1);
	vx = x_state(2);
	vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	c1 = ((px * px) + (py * py));
  if (fabs(c1) < ZERO_DETECTION) {
		c1 = ((c1 > 0) - (c1 < 0)) * ZERO_DETECTION; // avoid value close to zero - retain sign
	}
	c2 = sqrt(c1);
	c3 = (c1 * c2);

	//compute the Jacobian matrix
	Hj(0, 0) = (px / c2);
	Hj(0, 1) = (py / c2);
	Hj(1, 0) = -(py / c1);
	Hj(1, 1) = (px / c1);
	Hj(2, 0) = ((py * ((vx * py) - (vy * px))) / c3);
	Hj(2, 1) = ((px * ((px * vy) - (py * vx))) / c3);
	Hj(2, 2) = (px / c2);
	Hj(2, 3) = (py / c2);

	// display message if required	
	if (bDISPLAY) {
		cout << "  Jacobian Hj: " << Hj << endl;
		cout << "--- Tools: CalculateJacobian - End" << endl;
	}
	
  // return the result
	return Hj;
	
}
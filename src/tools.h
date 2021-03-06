#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class Tools {

public:

	/**
	* Constructor.
	*/
	Tools();

	/**
	* Destructor.
	*/
	virtual ~Tools();

	/**
	* A helper method to calculate RMSE.
	*/
	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	/**
	* A helper method to calculate Jacobians.
	*/
	MatrixXd CalculateJacobian(const VectorXd& x_state);

private:

	// define matrices
	VectorXd rmse;
	VectorXd residual;
	MatrixXd Hj;

	// define cartesian coordinates
	double px;
	double py;
	double vx;
	double vy;

	// define helper terms
	double c1;
	double c2;
	double c3;

	// boolean for display debug messages
	const bool bDISPLAY = true;
	const bool bDISPLAYDETAIL = false;

	// define constants
	const int NUM_RADAR_MEASUREMENTS = 3;
	const int NUM_STATES = 4;
	const int ZERO_DETECTION = 0.0001;	

	// define variables
	unsigned int vCount;

};

#endif /* TOOLS_H_ */

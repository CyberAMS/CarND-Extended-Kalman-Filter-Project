#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"

class MeasurementPackage {

public:

	long long timestamp_;

	enum SensorType{

		LASER,
		RADAR

	} sensor_type_;

	Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
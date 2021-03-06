#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <streambuf>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
	
  if (found_null != string::npos) {
		
    return "";
		
  } else if (b1 != string::npos && b2 != string::npos) {
		
    return s.substr(b1, b2 - b1 + 1);
		
  }
	
  return "";
	
}

// define constants
const bool bFILEOUTPUT = true;
const bool bDISPLAY = true;

// define file for redirecting standard output and append
ofstream out("out.txt", fstream::app);
streambuf *coutbuf = cout.rdbuf(); // save screen object

int main() {
	
  uWS::Hub h;
	
  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(string(data));
			
      if (s != "") {
      	
        auto j = json::parse(s);

        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
					long long timestamp;
					
					// reads first element from the current line
					string sensor_type;
					iss >> sensor_type;

					if (sensor_type.compare("L") == 0) {
						
      	  	meas_package.sensor_type_ = MeasurementPackage::LASER;
          	meas_package.raw_measurements_ = VectorXd(2);
          	double px;
      	  	double py;
          	iss >> px;
          	iss >> py;
          	meas_package.raw_measurements_ << px, py;
          	iss >> timestamp;
          	meas_package.timestamp_ = timestamp;
						
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		double roh;
      	  		double theta;
      	  		double roh_dot;
          		iss >> roh;
          		iss >> theta;
          		iss >> roh_dot;
          		meas_package.raw_measurements_ << roh, theta, roh_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
					
					// redirect standard output to file if necessary
					if (bFILEOUTPUT) {
						cout.rdbuf(out.rdbuf());
					}

					// display message if required
					if (bDISPLAY) {
						cout << "Main: onMessage - Start" << endl;
						cout << "  Time stamp: " << timestamp << endl;
						cout << "  Sensor type (string): " << sensor_type << endl;
					}					

          double x_gt;
					double y_gt;
					double vx_gt;
					double vy_gt;
					iss >> x_gt;
					iss >> y_gt;
					iss >> vx_gt;
					iss >> vy_gt;
					VectorXd gt_values(4);
					gt_values(0) = x_gt;
					gt_values(1) = y_gt; 
					gt_values(2) = vx_gt;
					gt_values(3) = vy_gt;
					ground_truth.push_back(gt_values);
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
					fusionEKF.ProcessMeasurement(meas_package);    	  

					//Push the current estimated x,y positon from the Kalman filter's state vector
					VectorXd estimate(4);

					double px = fusionEKF.ekf_.x_(0);
					double py = fusionEKF.ekf_.x_(1);
					double v1  = fusionEKF.ekf_.x_(2);
					double v2 = fusionEKF.ekf_.x_(3);

					estimate(0) = px;
					estimate(1) = py;
					estimate(2) = v1;
					estimate(3) = v2;
    	  
					estimations.push_back(estimate);

					VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = px;
          msgJson["estimate_y"] = py;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
					
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
					// display message if required
					if (bDISPLAY) {
						cout << "  Message: " << msg << endl;
						cout << "--- Main: onMessage - End" << endl;
					}					
					
					// set standard output to screen if necessary
					if (bFILEOUTPUT) {
						cout.rdbuf(coutbuf);
					}
					
        }
				
      } else {
        
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
			
      res->end(s.data(), s.length());
			
    } else {
			
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
			
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		
    cout << "Connected!!!" << endl;
		
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		
    ws.close();
    cout << "Disconnected" << endl;
		
  });

  int port = 4567;
  if (h.listen(port)) {
		
    cout << "Listening to port " << port << endl;
		
  } else {
		
    cerr << "Failed to listen to port" << endl;
		
    return -1;
		
  }
	
  h.run();
	
}
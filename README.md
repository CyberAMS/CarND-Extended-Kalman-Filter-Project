# Project: Extended Kalman Filter

This project has been prepared by Andre Strobel.

The goal of this project is to program an extended Kalman filter in *C/C++* to demonstrate [sensor fusion](https://en.wikipedia.org/wiki/Sensor_fusion). In this example the extended Kalman filter tracks a moving object using [LIDAR](https://en.wikipedia.org/wiki/Lidar) and [RADAR](https://en.wikipedia.org/wiki/Radar) measurements.

The following table shows an overview of the most important files:

| File                                          | Description                                                    |
|-----------------------------------------------|----------------------------------------------------------------|
| README.md                                     | This file                                                      |
| install-ubuntu.sh                             | Script to install uWebSocketIO                                 |
| data/obj_pose-laser-radar-synthetic-input.txt | Example input data file                                        |
| build/ExtendedKF                              | Compiled executable to run the extended Kalman filter          |
| build/out.txt                                 | Debugging output of the extended Kalman filter                 |
| src/main.cpp                                  | Source code of main function of extended Kalman filter project |
| src/FusionEKF.{h, cpp}                        | Source code of sensor fusion object                            |
| src/kalman_filter.{h, cpp}                    | Source code of extended Kalman filter implementation           |
| src/tools.{h, cpp}                            | Source code of tool object used by extended Kalman filter      |

---

## Content

1. Tool chain setup
    1. Gcc, Cmake, Make and uWebSocketIO
    1. Udacity Simulator
1. Input data definition
    1. Input data structure
    1. Input data files
1. Extended Kalman filter implementation
    1. Necessary equations
    1. Implementation in C/C++
1. Execution with given input data
    1. Commands to start the simulation
    1. Simulation results
1. Discussion

[//]: # (Image References)

[image1]: docu_images/01_02_center_2018_08_18_06_11_22_467.jpg
[image2]: docu_images/01_02_center_2018_08_18_06_11_22_467_cropped.jpg
[image3]: docu_images/01_02_center_2018_08_18_06_11_22_467_flipped.jpg
[image4]: docu_images/04_01_mse_loss_index.png
[image5]: docu_images/07_01_nm_bridge_stuck.jpg

---

## 1. Tool chain setup

### 1. Gcc, Cmake, Make and uWebSocketIO

This project requires the following programs:

* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
  * Works with Linux and Mac systems
  * Windows: Use Docker, VMware or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (although I wasn't able to get it working with the latest Ubuntu app in Windows 10)

### 2. Udacity Simulator

The extended Kalman filter program connects to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases) via [uWebSocketIO](https://github.com/uWebSockets/uWebSockets). The simulator is available for Linux, Mac and Windows.

## 2. Input data definition

### 1. Input data structure

The input data is either a LIDAR or RADAR measurement.

LIDAR measurements are structures the following way:

| Variable              | Value or Description                  |
|-----------------------|---------------------------------------|
| `sensor_type`         | L                                     |
| `x_measured`          | measured x position of object         |
| `y_measured`          | measured y position of object         |
| `timestamp`           | time in nano seconds since 01/01/1970 |
| `x_groundtruth`       | actual x position of object           |
| `y_groundtruth`       | actual y position of object           |
| `vx_groundtruth`      | actual x velocity of object           |
| `vy_groundtruth`      | actual y velocity of object           |
| `yaw_groundtruth`     | actual yaw angle of object (not used) |
| `yawrate_groundtruth` | actual yaw rate of object (not used)  |

RADAR measurements are structures the following way:

| Variable              | Value or Description                  |
|-----------------------|---------------------------------------|
| `sensor_type`         | R                                     |
| `rho_measured`        | distance to object                    |
| `phi_measured`        | angle to object                       |
| `rhodot_measured`     | change of distance to object          |
| `timestamp`           | time in nano seconds since 01/01/1970 |
| `x_groundtruth`       | actual x position of object           |
| `y_groundtruth`       | actual y position of object           |
| `vx_groundtruth`      | actual x velocity of object           |
| `vy_groundtruth`      | actual y velocity of object           |
| `yaw_groundtruth`     | actual yaw angle of object (not used) |
| `yawrate_groundtruth` | actual yaw rate of object (not used)  |

### 2. Input data files

The input data file contains LIDAR or RADAR measurements in rows. The variables inside a row are separated by tabulators. An example file can be found in [./data/obj_pose-laser-radar-synthetic-input.txt](./data/obj_pose-laser-radar-synthetic-input.txt).

Additional input data files can be generated with the [utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) which contains [Matlab](https://www.mathworks.com/products/matlab.html) scripts that can generate this data.

## 3. Extended Kalman filter implementation

### 1. Necessary equations

In this project the prediction steps of the standard Kalman filter for LIDAR measurements and the extended Kalman filter for RADAR measurements are identical, because we assume a linear motion model.

```C++
// predict state - linear model, i.e. same for LIDAR and RADAR (no need to use Jacobian of F)
x_ = F_ * x_;

// predict noise
Ft = F_.transpose();
P_ = (F_ * P_ * Ft) + Q_;
```

The update step of the standard Kalman filter for LIDAR measurements

```C
// calculate y for LIDAR measurement (linear model, standard KALMAN filter)
y_laser = z - H_ * x_;

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
```

### 2. Implementation in C/C++

XXX

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## 4. Execution with given input data

### 1. Commands to start the simulation

XXX

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

### 2. Simulation results

XXX

## 5. Discussion

XXX






















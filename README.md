# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

## Overview
The included source implements the extended Kalman filter in C++. The data folder contains simulated lidar and radar measurements for an object that travels in a figure eight around a sensor. The Kalman filter estimates the object's position and velocity. The metric used to measure the deviation from the ground truth is root mean square error (RMSE).

## Compilation
The source can be compiled without error using `cmake` and `make`.<br>
![Kalman filter cmd compile](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/CMD-compile.PNG "Kalman filter cmd compile")

The source can be compiled without error using Microsoft Visual Studio.<br>
![Kalman filter mvs compile](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/MVS-compile.PNG "Kalman filter mvs compile")

## Results for Included Data Sets
Data set 1 RMSE: 0.0651795 0.0605726 0.544212 0.544226<br>
Data set 2 RMSE: 0.185791 0.190311 0.474852 0.805316<br>
![Kalman filter results](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/Results.PNG "Kalman filter results")

## Results Visualization
Data set 1 is visualized using a chart in Microsoft Excel. The green line is the ground truth object location. The orange markers are lidar/radar measurements. The blue line is the EKF estimate of position.<br>
![Kalman filter results visualization](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/Visualization1.PNG "Kalman filter results visualization")

## 2-D Unity Visualizer
Important: A modified `kalman_tracker.py` script is included.
Usage: `python kalman_tracker.py src/ExtendedKF.exe`

To optimize the Kalman filter program for real-time use with the Unity Visualizer, the Python script adds a flag to the internal command line (subprocess). This instructs the filter program to utilize only the last 10 items of the input file. The RMSE reports in the Visualizer therefore correspond to the previous 10 measurements.

Below is the fused result of a run with Lidar and Radar enabled:<br>
![Unity visualization LR](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/viz-LR.PNG "Unity visualization LR")

Below is the result with only Radar enabled:<br>
![Unity visualization R](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/viz-R.PNG "Unity visualization R")

Below is the result with only Lidar enabled:<br>
![Unity visualization L](https://github.com/jwdunn1/CarND-Extended-Kalman-Filter-Project/blob/master/images/viz-L.PNG "Unity visualization L")

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. Sample inputs can be found in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

A debug version can be made using: `cmake -DCMAKE_BUILD_TYPE=Debug .. && make`

## Code Style

Conforms to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

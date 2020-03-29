# CarND-Path-Planning-Project

[![Youtube](http://img.youtube.com/vi/Aq-EBnPpHkY/0.jpg)](https://www.youtube.com/watch?v=Aq-EBnPpHkY "Pathplanning")
  (click to see Youtube Video)

## Objective
- Let a car autonomously plan path on highway based on given sensor fusion data (positions and velocities of other cars) and map data

## Approach 
- Based on given map data, change X-Y position data into S-D data
- Set target location to go (upto 100 meters in front of the car) 
- Also set taret velocity (default is slightly slower than speed limit)
- Temp waypoints are created and smoothed to quadratic using spline library (https://kluge.in-chemnitz.de/opensource/spline/)
- Create waypoints toward that point so that the acceleration does not exceed a threshold
- Based on sensor fusion data, change the target location (thus change the lane and distance/speed), by rule based algorithm: 
  - if there is no car in front, set target speed to close to the speed limit
  - if threr is a car in near front, keep the same speed of the car in front
  - if there is space in left lane or right lane, and there is no car near backward, change to the lane
  - not making another decision when changing the lane and soon after changing the lane
  - if there is no car in front in the center lane and no other car is approaching from back, return to the center lane

## Results

- The car could run the virtual highway for 3 miles without any trouble
- The car could keep close to the limit speed by changing lanes
- See the video https://youtu.be/Aq-EBnPpHkY


## Further improvements 
- Since this algorithm is based on simple rule based algorithm this cannot react to complicated/unecpected situations (ex. accidents)
- To make it complicated, applying score-based algorithms can be considered


---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run `install-ubuntu.sh`.
    
* Simulator [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  
  * To run the simulator on Mac/Linux, first make the binary file executable with the following command:
  `shell sudo chmod u+x {simulator_file_name}`


## Setup 

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.





# Path Planning Writeup

## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Compilation Instructions

1. mkdir build
2. cd ./build
3. cmake ..
4. make
5. ./path_planning

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Data Description

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


## Code Description
The kernel code for path planning can be seen in `src/main.cpp`. In `main.cpp` file, it includes some helper function, such as hasData, distance(calculate the distance between two points), ClosestWaypoint(find the closet landmark with the point), NextWaypoint, getFrenet(transform x,y position to s,d position), getXY(transform s,d position to x,y position). Besides, main function also in `main.cpp` file and it contains messaging service between the code and the simulator, and also to execute the path planning logic.

In the main function, it includes three parts function: prediction, behavior, trajectory.

### Prediction

The part of code which deals with the sensor fusion data. The data from external sensors provides information about the surroundings. Based upon this data, we can get to know if there are any cars in our surroundings, like a car in front of us going at lower speeds or if there is a car in the right or left lane. A distance of 30 meters is considered as a safe distance to maintain between other cars, so if there is a car in the same lane in front and the car is closing by and reaches less than 30 meters, then we predict that its no more safe to go at the current speed and in the current lane.

### Behavior

This is the most important part of the algorithm. Based upon the sensor data received and the prediction step, we should take a decision if we have to maintain the same lane when we find a car in front of us, and do we have to slow down or speed up or maintain speed. If there is any car in front and in the same lane, then free space is checked in left and right lane, before changing the lane. Cars on rear side of car on either sides of lane are also checked for safe lane change. If the lane change is calculated to be not safe, then the speed of the vehicle is reduced to maintain a safe distance from the front vehicle. For a safe acceleration and deceleration wihtout any jerks, the difference in speed is provided in small increments.

### Trajectory

Based on the speed, lane decison, past points and present car coordinates, this part of the code calculates trajectory. The coordinates are transofrmed to local car coordinates and spline library is used for calculation. Past two points and the farthest two points are used by spline to calculate the trajectory. The trajectory points in between are calculated by spline. The output of the spline is smooth during lane change as well.

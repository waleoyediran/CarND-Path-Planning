# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
The objective of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

#### Input
We are provided  the following a program inputs:
* The car's localization and sensor fusion data.
* A sparse map list of waypoints around the highway. 

#### Expectations
* The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
* The car should avoid hitting other cars at all cost 
* The car is expected to drie inside of the marked road lanes at all times, unless going from one lane to another. 
* The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
* Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

![Highway driving][image1]

[image1]: ./images/highway-driving.png "Highway driving"

## Reflection
#### Behaviour planning
The challenge is to create an intuition of what the car should do to achieve its goal of driving around the course, while
observing the environment.
The steps I have chosen to achieving that is described as follows.

* Track the other cars within the `SAFE_DISTANCE` around your car and determine if there is a car in your lane ahead of you, on the lanes to your right and to your left.

```
double o_car_velocity_x = fusion_data[3];
double o_car_velocity_y = fusion_data[4];
double o_car_velocity = sqrt(o_car_velocity_x * o_car_velocity_x + o_car_velocity_y * o_car_velocity_y);
double o_car_s = fusion_data[5]; // longitudinal position of the car in frenet coordinates
double o_car_d = fusion_data[6]; // lateral position of the car in frenet coordinates

int o_car_lane;

if (o_car_d <= 4) {
    o_car_lane = LEFT_LANE;
} else if (o_car_d > 4 && o_car_d < 8) {
    o_car_lane = MID_LANE;
} else if (o_car_d >= 8) {
    o_car_lane = RIGHT_LANE;
}

o_car_s += TIME_INC * o_car_velocity;

double dist_btw_cars = o_car_s - car_s;
if (dist_btw_cars > (-SAFE_DISTANCE + 10) && dist_btw_cars < SAFE_DISTANCE) {
    if (lane == o_car_lane && dist_btw_cars > 0) {
        car_in_front = true;
    }
    if ((lane - o_car_lane) == 1) {
        car_left = true;
    }
    if ((lane - o_car_lane) == -1) {
        car_right = true;
    }
}
```

* Make a driving decision based on the cars in the lanes around your car. My simplistic approach, decides to stay in lane, change, lane right, change lane left. 

```c
if (car_in_front) {
  if (lane > LEFT_LANE && !car_left) {
    lane--;
    cout << "Change lane left" << endl;
  } else if (lane < RIGHT_LANE && !car_right) {
    lane++;
    cout << "Change lane right" << endl;
  } else {
    cout << "Reduce speed " << car_velocity << endl;
  }
  car_velocity -= velocity_increment;
} else if (car_velocity < max_velocity) {
  car_velocity += velocity_increment;
  cout << "Increase speed " << car_velocity << endl;
}
```

#### Trajectory Generation
After making a driving decision, the next step is to generate a trajectory in driving coordinates that should be visited by the car.
3 uniformly spaced points in the choice lane ahead is selected. This points are augmented with one previously transversed point.
These points are used to create smooth drivable path function to the goal position.

Using the path function, we create 50 coordinates that represent the the path uniformly spaces to the target position and velocity.

```
double x_inc = goal_x / (goal_distance / (car_velocity * TIME_INC));
for (int i = 1; i <= 50 - curr_path_size; i++) {
  double xp = x_inc * i;
  double yp = sp(xp);
  double point_x = ref_x + (xp * cos(ref_yaw) - yp * sin(ref_yaw));
  double point_y = ref_y + (xp * sin(ref_yaw) + yp * cos(ref_yaw));

  next_x_vals.push_back(point_x);
  next_y_vals.push_back(point_y);
}

``` 

### Improvements
* Create a Finite stet machine to represent driving behavior.
* Create a cost function that assigns weight to average speed of the cars in the lane to optimise drive time.
* Create probable trajectories of the other cars to anticipate their driving intentions.

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
### Simulator.
The project requires the Udacity Self Driving Car Nanodegree Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
---

> The video below shows the car driving in the simulator highway
 
 [![Highway Driving][image2]](https://youtu.be/1Hnz7K5UkgY)
 
 [image2]: ./images/video-planning.png "Highway Driving"
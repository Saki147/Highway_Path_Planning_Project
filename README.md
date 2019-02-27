# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

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

## Reflection 

![image1](./Images/path_planning.jpg "Path planning flowchart")

The ego car's localization data is provided by the simulator, and the sensor fusion data show the state of vehicles 
nearby. With the localization of the ego car, the relative location of the cars nearby, as well as speed of both ego 
and nearby vehicles, I can predict the future (next step) state of the cars. Therefore, with the goal of driving as 
close as to the speed limit without collisions, the behaviour of the car can be determined. Then, the map waypoints can 
help to generate the trajectories. Here, the **spline function** is used to generate a smooth trajectory which is 
introduced in **Tips** section later. The planned path is shown by green points in the simulator. 


### Prediction
The prediction component will help to predict where the vehicle will be in future, project s value out in time. Here,
 vehicles nearby are assumed driving straight in the same lane, so that their new location will be calculated as below:
```bash
check_car_s += ((double)prev_size*0.02*check_speed);
```
The ego car's predicted localization is the end of the previous path ```end_path_s```.

For each iteration, the state boolean for car_left lane, right_right lane, and car_ahead lane are set to be false 
initially.
```bash
bool car_left = false;
bool car_right = false;
bool car_ahead = false;
```
If the vehicle nearby is on each lane, and near the ego car, they will be activated to be true.
```bash
if(d > 0 && d < 4) {
    check_car_lane = 0;
} else if(d > 4 && d < 8) {
    check_car_lane = 1;
} else if(d > 8 and d < 12) {
    check_car_lane = 2;
}

```
If the left, right, current lane has car very close to the ego car, normally within 30m or 25m, the lanes are marked 
as true. If there is a car ahead, the speed of the nearest car will be recorded for further use. The cost function is
 used to get a max cost (```max_cost_l``` / ```max_cost_r```) inversely proportional to the distance between ego car 
 and  some far away cars on the left lane and right lane. Therefore, when the car is at the middle lane with a car 
 ahead, and there is no cars nearby at left or right lane, the car will determine to change to the lane with less 
 cost, rather than just change to the left lane, which makes the car looks further.

```bash
if(check_car_lane == lane) {
    //A vehicle is on the same lane and check the car is in front of the ego car
    diff_car_s = check_car_s - car_s;
    car_ahead |= diff_car_s > 0 && diff_car_s < 30;
    if (car_ahead && diff_car_s < min_car_diffs ){
        min_car_diffs = diff_car_s;
        check_speed_ahead = check_speed;
    }
} else if((check_car_lane - lane) == -1) {
    //A vehicle is on the left lane and check that is in 30 meter range
    diff_car_s = fabs(car_s - check_car_s);
    car_left |= diff_car_s < 25;
    cost_l = 1/ (1+ exp(diff_car_s*diff_car_s)); //cost of change to the left lane
    if (max_cost_l < cost_l){
        max_cost_l = cost_l; //record the max cost for left lane change
    }
} else if((check_car_lane - lane) == 1) {
    //A vehicle is on the right lane and check that is in 30 meter range
    diff_car_s = fabs(car_s - check_car_s);
    car_right |= diff_car_s < 25;
    cost_r = 1/ (1+ exp(diff_car_s*diff_car_s)); //cost of change to the right lane
    if (max_cost_r < cost_r){
        max_cost_r = cost_r; //record the max cost for left lane change
    }
}
```

### Behavioral Planning
The behavioral planning component determines what behavior the vehicle should exhibit at any point in time.
For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.

Here, if there is a car ahead, and no nearby cars on the left or right lane, the car will decide to change to left 
or right lane; while reduce the speed until it has the same speed as the car ahead when it can change lane.
```bash
if(car_ahead) {
    if(!car_left && lane > 0 ) {
      if (lane == 2| max_cost_l <= max_cost_r){
        lane--;
      }
    } else if(!car_right && lane !=2 ) {
        lane++;
    }else {
        if (ref_vel > check_speed_ahead ){
            ref_vel -= speed_diff; //reduce the car speed by speed_diff if it's faster than the car ahead
        } else{
            ref_vel = check_speed_ahead; //follow the front car with the same speed
        }
    }
} else if(ref_vel < max_vel ){
ref_vel += speed_diff; //add the speed when there's no car ahead and the speed is under the limit
}
```

### Trajectory Generation
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best 
for executing this behavior. In this case, it should be smooth and avoid collision.

I use the ```ptsx``` and ```ptsy``` to record the next location with previous path and Three future points 
called ```next_wp0```, ```next_wp1```, and ```next_wp2```. In addition, ```ref_x```, ```ref_y```, and ```ref_yaw``` 
as the ego car's 
state in global 
coordinate.  
```bash
vector<double> ptsx;
vector<double> ptsy;

//Refrence x,y, and yaw states
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);

```
```
// Setting up target points in the future.
// the car is at its lane center
vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```
           
The target points are stated in global coordinates and will be shifted, and rotated to the car's local coordinates 
for the ease of calculation. 
```bash
for ( int i = 0; i < ptsx.size(); i++ ) {
  double shift_x = ptsx[i] - ref_x;
  double shift_y = ptsy[i] - ref_y;

  //for the car's coordinates
  ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw); //car's heading direction
  ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw); //car's perpendicular direction
}
```      

The previous path contributes to the next path for a good connection to the past, and avoid jerk.  
```bash
vector<double> next_x_vals;
vector<double> next_y_vals;

//For the smooth transition, we are adding previous path points
for ( int i = 0; i < prev_size; i++ ) {
  next_x_vals.push_back(previous_path_x[i]);
  next_y_vals.push_back(previous_path_y[i]);
}
```
The spline function is used to generate a smooth trajectory. 
```bash
// Create the spline.
            tk::spline s;
            s.set_points(ptsx, ptsy);
```
Each point's local coordinates on the spline for every 0.02s time step are get, and are transformed to the global 
coordinates. ```next_x_vals``` and ```next_y_vals``` save the generated trajectories. 

```bash
// Calculate distance y position on 30 m ahead.
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x*target_x + target_y*target_y);

double x_add_on = 0;

// Fill up the rest of our path planner after filling it with the previous points, here always output 50
//points.
for( int i = 1; i < 50 - prev_size; i++ ) {

  double N = target_dist/(0.02*ref_vel/2.24); // N = the number of points, (1m/s = 2.24 MPH)
  double x_point = x_add_on + target_x/N; // x coordinate for point i
  double y_point = s(x_point); // y coordinate for point i

  x_add_on = x_point; // new x coordinate

  double x_ref = x_point; // record the x local coordinate as x_ref
  double y_ref = y_point; // record the y local coordinate as y_ref

  //Rotate back to normal after rotating it earlier
  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw); //rotate the coordinate
  y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw); //rotate the coordinate

  x_point += ref_x; // shift the x coordinate to the global coordinate
  y_point += ref_y; // shift the y coordinate to the global coordinate

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}

```


## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The 
units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector 
going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential 
and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, our project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests


We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./


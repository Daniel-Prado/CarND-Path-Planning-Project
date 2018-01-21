# Project: Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

#### Daniel Prado Rodriguez
#### Feb'2017 Cohort
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## Reflection

### General Approach
In this project I have tried to apply most of the concepts discussed in the Path Planning lessons, hence, I have tried to implemented all the different modules of a Path Planning system in a self-driving car, and the structure of classes of the code reflects that modularity:

* **VEHICLE**: this is the class that represents all the cars as objects, for both the Ego car, and the cars detected in the sensor fusion data (stored as a vector of Vehicle class objects. The status of a Vehicle is represented by `s`, `s_dot`, `s_ddot`, `d`, `d_dot` and `d_ddot`.
of the car it represents.The class has also another utility: it serves to define paths or trajectories, where every point in the path is an instance of Vehicle. For example, the predicted positions over a time span for a given car in the sensor fusion data are stored also as a vector of Vehicle objects.
Note that all coordinates are stored exclusively in Frenet. This was a difficult decission, but I followed the advice of teachers to use Frenet to ease calculations. The drawback is that we cannot trust the XY coordinates given by the simulator. Instead, we based  new trajectory calculations at its start always based on the points of the previous trajectory.
* **ROAD MAP**: this module loads the waypoint coordinates for the track given in a text file, and based on those, it calculates SPLINES that will serve later to transform any Frenet coordinates into XY coordinates. The reason is that the provided GetXY method doesn't work very well around the waypoints, causing 'jumps' and jerk when trying to feed the XY coordinates into the simulator. Instead, ALL the calculations are done in Frenet, and only converted to XY map coordinates before feeding the simulator.
* **BEHAVIOR PLANNING**: this module provides all the necessary 'brain' functions for the path planning algorithm (which is also implemented at a high level in the main.cpp file). The behaviour planner implements several functions that we will discuss later, such as:
  - (Public) Data fussion vehicles prediction generation.
  - (Public) Plan Decission (function make_plan)
  - State Machine (successor states function).
  - Possible collision detection.
  - Cost Function calculation for a possible maneuver.
  - Find vehicles ahead in a lane.
  - New trajectory Generation.
  - etc.
* **TRAJECTORY GENERATOR** : this module calculates a path for the ego vehicle using SPLINES.

### Sequence of Operations
This details the sequence of operations, at a high level, implemented in `main.cpp`:
1) [Update ego vehicle position along the trajectory](src/main.cpp#L253-L271): given the number of steps the car has moved along the previous trajectory (given by the simulator), we update our frenet position stored in memory, and get rid off the already consumed leg.
2) [Refresh previous path](src/main.cpp#L275-L281): here we get rid off the part of the local Frenet previous trajectory that the car has already run.
3) [Load Sensor fussion data](src/main.cpp#L284-L310):  We load the other vehicles frenet coordinates into a vector of Vehicle objects. Note that in this implementation we work on a fresh new vector every cycle, hence we don't have history of the previous status.
4) [Predict vehicles movement and decide new Plan](src/main.cpp#L312-L323) based on those predictions.
5) [Get updated XY cartesian trajectory](src/main.cpp#L326-L350) based on the calculated plan path (frenet) and send it to the simulator.

### Predictions
The prediction of the movement of sensor fussion vehicles is done in a simple way, just taking into account the instantanious (current) speed of the vehicle along the `s` coordinate.
Also, I was forced to ignore the speed in the `d` coordinate. This is because the cars oscillate quite a lot in the `d` direction while keeping the lane, hence making prediction based only on the d_dot instant speed leads to wrong conclussions (such as lane changes that in practice happen very rarely).  This could be improved in a future version analyzing the positions along time, applying some low-pass filter, or even using some more sofisticated predictor (Naive Bayes or other machine learning algorithm).
The code can be checked [here](src/BehaviorPlanner.cpp#L16-L35) and [here](src/Vehicle.cpp#L24-L32).

### Plan Decission
The code for the plan path decission making at high level is implemented in this [function](src/BehaviorPlanner.cpp#L38-L91). Basically, for the given state and lane, we get the list of successor states. For every successor state, we calculate a possible trajectory. For every trajectory, we calculate its cost, and finally, we determine the next cycle target lane, state, and path trajectory that minimizes the cost.
There is one exception: we want that once a decission is made to execute a lane change, we wait until the maneuver is finished before evaluating other options.

Note that the __state machine__ implemented has only 3 states: KL(Keep Lane), (LCL)Lane Change Left and (LCR)Lane Change Right. A more sofisticated state machine including the Prepare LCL and Prepare LCR states is left for future improvements.


### Target position & Trajectory Calculation
As mentioned before, I used the provided SPLINE library to generate trajectories. The code can be checked [here](src/TrajectoryGenerator.cpp#L37-L134). The first two points of the spline are taken from the previous trajectory, and then the other 3 points are equally spaced every 30m, with a 'd' coordinate equal to the target lane of the trajectory. This separation has empirically shown to be adequate to ensure low acceleration and jerk, as seen in the teacher's Q&A session. I preferred this option of working exclusively in Frenet instead of using Car XY coordinates as shown in the Q&A. The advantage of using Frenet, is that we can space the points along the S coordinate exactly using the law's movement of phisics, taking into account the speed and the acceleration of the car, instead of doing that in an 'x' axis and projecting over the 's' curve.
Once the spline is calculated, note that the positions along the trajectory are calculated based on 'goal' or 'target' position, that will depend on the lane and the position of the leading car. This 'goal' is calculated [here](src/BehaviorPlanner.cpp#L349-L374).

### Cost Function
The cost function implemented is composed of several terms. Check the code [here](src/BehaviorPlanner.cpp#L104-L144):
* __Collision__: is a collision is detected, the cost is maximum (100,000) and other terms are ignored.
* __Speed__: the speed at the target position of the trajectory is evaluated and compared with the maximum possible speed. The cost is proportional to that difference.
* __Traffic__: (cost_busy_lane in the code): the traffic ahead of the target lane is evaluated, even if it doesn't immediately affect the current trajectory. The cost is calculated as a sigmoid function based on the distance to the ahead traffic.
* __Central_lane__: This simply slightly favors the central lane over the left and right lanes, in situations where the cost is the same for the three of them (all lanes clear for example). I do this because the central lane has always 2 options to maneuver (left and right).



# Project Instructions:
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

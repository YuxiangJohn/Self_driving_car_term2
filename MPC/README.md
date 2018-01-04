# CarND-Controls-Model Predictive Control (MPC)

<img src="./graph/result.gif">

---

## MPC Description
Model predictive control (MPC) is an advanced method of process control. Importantly, MPC is not only the pursuit of control, but also the pursuit of optimal control. Compared with the traditional PID controller, it considers whether the most economical way to control. 
MPC is an optimization control problem that is devoted to decomposing optimal control problems of longer time span or even infinite time into several shorter time spans or limited time span, and to some extent still pursuing the optimal solution. Under the constraints to reach the goal, MPC finds an way to lower (optimize) the value of cost function. 

In this project, I use MPC to control a car in a simulator, visiualize the path and the predicted waypoints. The optimization considers only a short duration's worth of waypoints. MPC produces a trajectory for that duration based upon a model of the vehicle's kinematics and a cost function based mostly on the vehicle's cross-track error (roughly the distance from the track waypoints) and orientation angle error, with other cost factors included to improve performance.  

I choose N = 10 and dt = 0.1. A larger N value leads to a slower simulation as the computation of the future value increases. dt = 0.1 is the time interval between each predicted point. The predicted time in future is 1s (N * dt = 1s). After playing with other values, I found N = 10 and dt = 0.1 worked better. Before polynomial fitting, the waypoints are transformed from unity coordinates to the vehicle coordinates. I add latency (100ms) to the process so that it can simulate the actual situation.

### Kinematics model
The car kinematics model is described as following:

| MPC state    | Description |
| :--------   |  :-----  | 
| px          | car's position in x coordinate   | 
| py         | car's position in y coordinate    |
| psi           | car's heading direction       | 
| v           | car's velocity       |
| cte           | cross track error       | 
| epsi           | orientation error of the car |

The following equation calculate the states in next timestep base on the states in current timestep.

<img src="./graph/eq.png">

### Actuators
|Actuator| description |
|:----| :----|
| steering angle | range:[-25, 25] (degree) |
|throttle| speed control:[-1, 1] (from full break to full throttle)|

###

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.
* Fortran Compiler
* Ipopt > v3.12.7
* CppAD
* Eigen
There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`. 















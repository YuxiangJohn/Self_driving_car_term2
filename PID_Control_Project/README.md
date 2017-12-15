# CarND-Controls-PID

<img src="./graph/result.gif">

---

## PID system

<img src="./graph/system.PNG" width="50%" height="50%">

In this PID system graph, we can see the PID is consist of three parts and contribute to the input of gain. In this project, the input is the steering angle.
The effect of three PID parameters are described as following: 

* Proportional gain - Kp: multiplied by cross-track error (cte), a pure proportional term computes the proportional to the cte which will lead to the change of steering value. If the cte is larger, the steering value is larger too. But a pure P controller is unstable with the overshoot problem. In this case, the car will always oscillates in the lane.
 
* Derivative gain - Kd: the cte is multiplied by the derivative of cte (the rate of change). A D controller plays a role to decrease the overshoot problem. But it has no effect to cte. If we D controller, the osillation will decrease.
 
* Integral gain - Ki: the cte is multipied by the intergral of cte. It aims at solve the drift problem by the systematic bias, which means the zero steering doesn't lead to a straight trajectory. By accumulating the cte, I controller can produce a big value to change the final gain in the system when the integral is big. After adding the I controller, the cte will decrease and no drift happen.


The steer value is calculated as the equation:
steer_value = -Kp * cte - Kd * diff_cte - Ki * int_cte

In this project, I used: (Kp, Ki, Kd) = (0.2, 0.004, 3.0). Starting with all three parameters equals to 0, I tuned the Kp firslty. I increased the Kp until the 0.2 when the cte is fairly small. Secondly, I increased Ki until the overshoot problem disappeared a lot when Ki is 0.004. Finally, I set Kd equals to 3.0 and it solved the errors which still existed. These parameters can run the car safely within the lane with a speed around 30mph.

### P, I, D Analysis

| PID gain    | Overshoot | CTE|
| :--------:    |  :-----:  | :------:  |
| Kp          |Increase       | Decrease    |
| Ki         | Increase     | Decrease    |
| Kd           | Decrease      | No Change   |


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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Reference

1. https://innovativecontrols.com/blog/basics-tuning-pid-loops
2. http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID#7
3. https://www.allaboutcircuits.com/technical-articles/an-introduction-to-control-systems-designing-a-pid-controller-using-matlabs/
4. https://www.youtube.com/watch?v=4Y7zG48uHRo








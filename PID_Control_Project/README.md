# CarND-Controls-PID

![model](./graph/system.png)
---

## PID system
A PID controller is actually a three part system: 

* Proportional compensation: the main function of the proportional compensator is to introduce a gain that is proportional to the error reading which is produced by comparing the system's output and input.  
 
* Derivative compensation: in a unitary feedback system, the derivative compensator will introduce the derivative of the error signal multiplied by a gain Kd
Kd
.  In other words, the slope of the error signal's waveform is what will introduced to the output. Its main purpose is that of improving the transient response of the overall closed-loop system.
 
* Integral compensation: in a unitary feedback system, the integral compensator will introduce the integral of the error signal multiplied by a gain Ki
Ki
.  This means that the area under the error signal's curve will be affecting the output signal.  We will prove this later, but it is important to note that this facet of the controller will improve the steady-state error of overall closed-loop system.

In this project, I used: (Kp, Ki, Kd) = (0.2, 0.004, 3.0). These parameters can run the car safely within the lane.

### P, I, D Analysis

| CL RESPONSE     | RISE TIME| OVERSHOOT | SETTLING TIME| S-S ERROR |
| :--------:   | :-----:   |  :-----:  | :-----:  | :------:  |
| Kp         | Decrease   |Increase    | Small Change   | Decrease    |
| Ki         | Decrease  | Increase   | Increase   | Decrease    |
| Kd         | Small Change   | Decrease    | Decrease   | No Change   |


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








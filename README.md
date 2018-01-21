# CarND-Controls-MPC

Self-Driving Car Engineer Nanodegree Program

In this project I'll implement Model Predictive Control to drive the car around the track, similar to previous projects. This time, however, I'm not given the cross track error. I'll have to calculate that myself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

---

[image1]: ./model_equations.png "Model Equations"


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Rubric talking points

* Student describes their model in detail. This includes the state, actuators and update equations.

Here is the model that I used:

![alt text][image1]

The variables above represent the vehicle's x and y coordinates, orientation angle, velocity, cross-track error, and psi error, respectively. dt is how much time elapses between actuations. The outputs are acceleration and steering angle.

* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

N is the number of timesteps in the horizon. dt is how much time elapses between actuations. For example, if N were 20 and dt were 0.5, then T would be 10 seconds. N=10 and dt = .1 were originally suggestions from the SDC Slack channel. When playing with these values the dt value being higher would inhibit responsiveness. The model slowed down when N was too large.

* A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoint coordinates are preprocessed using vector transformation equations in order to be in vehicle coordinates:

  `
  for (int i = 0; i < ptsx.size(); i++) {
      double shift_x = ptsx[i] - px;
      double shift_y = ptsy[i] - py;

      ptsx[i] = (shift_x * cos(-psi) - shift_y * sin(-psi));
      ptsy[i] = (shift_x * sin(-psi) + shift_y * cos(-psi));
  }
  `
  
From these transformed coordinates the polyfit() function is used to calculate a forth-degree polynomial line, drawing the yellow line denoting the path the vehicle should ideally follow. Because we are now calculating from the perspective of the vehicle the x, y and psi are now zero'd out, simplifying the following calculation taken from the quiz

 `
  double cte = polyeval(coeffs, x) - y;
  double epsi = psi - atan(coeffs[1]); 
  `

* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Before accounting for latency the vehicle immediately veered off the road in the simulations. For latency I predicted values using the previously mentioned model starting from the current state and dt = .1. The resulting state from the prediction was the new initial state for MPC.

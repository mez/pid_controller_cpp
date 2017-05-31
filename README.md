PID Controller in C++11
---

### Simulator Settings Used
* Screen Resolution: 640x480
* Graphics Quality: Fastest

### Reflection
A PID controller is relatively simple to implement, the
challenging part is choosing proper coefficients (Kp, Ki, Kd) 
that stabilize the system as a whole.

* P = Proportional (I like to think of it as the Present)
* I = Integral (I like to think of it as the past)
* D = Derivative (I like to think of it as the future)

Each time step, the simulator gives you CTE (cross track error), 
speed, and steering angle. The CTE is how far the car drifts from
the center of the road. The mission was to adjust the coefficients
(Kp, Ki, Kd) so that it minimized the CTE.
 
I've only implemented a controller for the steering angle and kept
the speed constant. Hence my controller would calculate
the steering angle as follows:

```
double PID::steering() {
  double steer = -Kp*p_error - Ki*i_error  - Kd*d_error;

  if (steer < MIN_STEER)
    steer = MIN_STEER; //-1

  if (steer > MAX_STEER)
    steer = MAX_STEER; //1

  return steer;
}
```

* `-Kp*p_error` is responsible for keeping track of the present error.
It responds in proportion to the CTE.

* `-Ki*i_error` is the past, this is responsible for correcting system
biases. 

* `- Kd*d_error` is responsible for keeping track of the future. 
How will the error change in the future? This is needed
mostly to reduce oscillation that is usually caused by
having just a P controller with high `Kp` gain.

Since the simulator calculates the CTE for me, I can handle the rest
and compute the needed errors as follows:

```
void PID::UpdateError(double cte) {
  p_error = cte;
  if (prev_cte == numeric_limits<double>().max())
    prev_cte = cte;

  d_error = cte - prev_cte;
  prev_cte = cte;

  i_error += cte;
}
```

### How the hyperparameters were chosen.

The coefficients where chosen using a manual twiddle algorithm.
I first started with just the `Kp`. I reduced the speed
and set `Kp` such that I had min zig-zag. Then I introduced `Kd`
to reduce the zig-zag even more. At this point, I'd slowly bump
the speed and observe the car. If it starts to zig-zag, I'd repeat
the process. `Ki` seems to only make the stability worse, do 
I left it at zero. This in essentially made my PID controller a PD
controller!

### Future improvements

1. Learn how to develop a decent car model to automate the tuning
outside of the simulator.
    * I could of automated the twiddle algorithm using the
    simulator but it was faster to just do twiddle manually.
2. Improve the PID controller to handle speed.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

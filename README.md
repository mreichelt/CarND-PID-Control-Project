# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


# Implementing a PID controller

To be able to steer a vehicle so that it should drive as close to a reference lane as possible,
given a cross track error (CTE) that describes how far away the vehicle is from the reference location
we can implement a controller.

At first, we can try to steer the vehicle with a simple P controller, where we use a driving angle defined as:

```cpp
double steer_value = -Kp * cte;
```

The problem with this controller is that it will heavily oscillate around the true location and never steer
back when reaching its target location. Here a video of my implementation, where you will see that the vehicle will never
reach the center of the line and finally gets off track (parameter chosen manually):

[P controller with Kp=0.2](001_P0.2.mp4)

To tackle this problem we can add a differential part to steer back when the error gets smaller. Now we have a PD controller.
This one is already able to drive around the track. Again the parameters were chosen manually.
Formula now:

```cpp
double diff_cte = cte - last_cte;
double steer_value = -Kp * cte - Kd * diff_cte;
```

[PD controller with Kp=0.2 and Kd=3](002_P0.2_D3.0.mp4)

Finally, if a system has systematic bias (e.g. always drives 1m left from the desired location) neither P or PD controller
can handle this. For that we need an integrational part `int_cte` which sums up all previous errors:

```cpp
int_cte += cte;
double diff_cte = cte - last_cte;
double steer_value = -Kp * cte - Ki * int_cte - Kd * diff_cte;
```

I recorded no video of this because we'll later discover that the simulator of the Udacity CarND project has no systematic bias.


## Twiddling the parameters

It turns out a PID controller is really simple to implement – it's configuring the parameters which is harder.
I implemented a twiddling routine which tries to increase/decrease each parameter one by one and check each time
if that lead to an overall improvement.

I decided to use only one track run for the complete twiddling process, which in retrospective was a bad idea because:

- This requires each twiddling step to not break the run of the vehicle. If one step fails the complete process fails.
- Different segments of the track are easier/harder to steer. This may lead to situations where an actual parameter
improvement will not be accepted because the track was naturally harder at one segment, and vice versa.
- The implementation is harder to do, because the twiddling algorithm has to be broken up into many steps that need to be
executed on each step.

If I would do this project again I would go for a different approach: restarting the simulation again each time a
parameter would change.

### My learnings:

- If you can, twiddle with a constant scenario - e.g. by restarting the run from scratch!
- Manually tweak the parameters at first so they are not far off (so the vehicle drives at least one track).
- Change the `dp` values that describe how strong a change of a parameter is. For example, I had to use a really small
`dp` value for `Ki` because larger changes would break the run immediately.

### Final parameters + video

Now enough talking, let's see the results!
Final parameters:

```cpp
Kp = 0.315775;
Ki = 0;
Kd = 3.54665;
```

[Final PID controller](003_PID_final.mp4) (which actually is a PD controller)


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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

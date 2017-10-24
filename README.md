# PID controller

## Introduction

A PID controller is implement in C++ to maneuver the the self-driving car around the Udacity lake track. The simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

The project's rubric can be found [here](https://review.udacity.com/#!/rubrics/824/view)

The simulation video on [youtube]()

[//]: # (Image References)
[image1]: ./images/.png "PID"

![alt text][image1]

## Discussion

The steering angle is calculated from the PID equation:

steering = -Kp * CTE - Kd * CTE' - Ki * Σ(CTE)

Where,

P = -Kp * CTE, is the Proportional term

D = - Kd * CTE', is the Differential term

I = - Ki * Σ(CTE), is the Integral term

The *Proportional term (P)* is responsible for the angle that the car intercepts the reference trajectory. However, without the aid of the other two terms, P alone gets the vehicle into a marginally stable condition in which the car oscillates around the reference condition. Eventualy the car will never converge to the reference.

The *Differential term (D)* is responsible for the interception of the reference condition. It takes into account both the CTE of the previous step as well as the current CTE. DIfferentiates them over time and adjust the interception angle that is able to eventually converge to the reference condition.

The *Integral term (I)* is responsible for the elimination of possible bias due to missalignment of the wheels or any other implementation flaw of the car. If only the other two terms were to be implemented, the car would converge to the bias instead of the reference condition. I, takes into account all previous CTEs, integrates them over time, eliminates the bias, and allows the car to converge to the reference condition.

*Is it what you expected?*

*Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.*

*Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!*

## Installation
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

## Important Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
      * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
      * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. Clone this repo
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
* On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./pid`

```
$ mkdir build && cd build
$ cmake .. && make
$ ./pid
```

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

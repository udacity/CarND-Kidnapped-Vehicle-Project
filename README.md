# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

All you will submit is your completed version of `particle_filter.cpp`, which is located in the `src` directory.

## Project Introduction
In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

Your particle filter should use this data to maintain a best guess (and associated uncertainty) for the vehicle's x position, y position, and yaw.

## Getting Started
Once you have this repository on your machine, `cd` into this directory and run the following commands in your terminal:

```
> ./clean.sh
> ./build.sh
> ./run.sh
```

> **NOTE**
> If you get any `command not found` problems, you will have to install 
> the associated dependencies (for example, 
> [cmake](https://cmake.org/install/))

If everything worked you should see something like the following output:

```
.
.
.
Time step: 2444
Cumulative mean weighted error: x 0 y 0 yaw 0
Runtime (sec): 0.187226
This is the starter code. You haven't initialized your filter.
```



## Directory Structure and File Information
```
project
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   particle_output.txt
|   README.md
|   run.sh
|
|___data
|   |   control_data.txt
|   |   gt_data.txt
|   |   map_data.txt
|   |
|   |___observation
|       |   observations_000001.txt
|       |   ... 
|       |   observations_002444.txt
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
    |   print_samples.cpp
```
### project

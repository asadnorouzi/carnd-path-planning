# Path Planning Project for Highway Driving

## Overview

This project implements a path planning pipeline for a self-driving car to safely drive in a highway. The car must maintain its speed within the legally allowed speed in the highway and must safely drive and change lane where necessary. It must also adjust its speed while avoiding a sudden jerk.

The implementation is tested in a simulator provided by Udacity as part of the Udacity's Self-Driving Car Engineer Nanodegree Program!

There are three main steps to implement a smooth path planner:
1. Predicting the cars behavior
2. Planning and making a decision (e.g changing lane) based on the predicted world.
3. Creating an optimal trajectory to fulfill and complete the decision made in the previous step. The Jerk Minimizing Trajectory method is used for this step.

## Step 1 - Prediction

This step is implemented in the prediction.cpp file. It used the sensor fusion data to predict the other cars behavior. It is to see if there will be cars in the front, left, and/or right of our car in the near future. It is also to see if there will be a safe gap for lane changing.

## Step 2 - Behavior Planning

This step is implemented in the planner.cpp file. It will decide a behavior from the following choices:
- Keep lane and maintain speed
- Keep lane and adjust speed (decrease or increase)
- Change lane to the left (if it is safe)
- Change lane to the right (if it is safe)

## Step 3 - Creating an Optimal Trajectory

This step is implemented in the jmt.cpp file. The trajectory is generated using jerk minimizing method and spline. The trajectory comprises 50 waypoints taking into consideration the previous trajectory as well. The previous trajectory helps controlling the jerk rate.

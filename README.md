# Robot Localization using Particle Filters
By: Anmol Sandhu, Luke Witten, Richard Li

In your ROS package create a README.md file to hold your project writeup. Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

    What was the goal of your project?
    How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

# Project Goal
The goal for this project is essentially to create a particle filter algorithm for localizing a robot in a known map. Our algorithm's particles should converge as the robot moves around and collects data via its LIDAR scanner, from which we can then estimate the *true* locaton of the robot.This project can be used for a multitude of purposes, including obstacle avoidance and wayfinding, because knowing where the robot is is vital to figuring out where the robot needs to go.


# Implementation
Our implementation contains multiple parts. The block diagram below describes how they interact with each other. 

Functionally, our particle filter behaves as follows:

1. We initialize a bunch of points in space around our robot. 
2. We take the laser scan that the robot's LIDAR sensor gives us an project it onto each of the points, computing a "score" for each point that is proportional to how well the laser scan matches with the objects around it. 
3. We resample our points such that there tends to be more new particles around the particles with higher scores.
4. We allow the robot to move and give us new laser scan data.
5. Rinse and repeat steps 2 - 4 until the points converge onto a single point. 

But we'll go into more detail about each step. 


# Design Decision
One key design decision we had to make was related to our implementation of resampling. There are a number of ways to create new points around the points that were the best from the previous sampling. 

# Challenges

# Improvements

# Interesting Lessons


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

## Updating the particles

1. `initialize_particle_cloud`
   This functino is responsible for creating all of our initial particles. We do this by creating distributions from which to sample for the x, y, and theta of our particles. We then sample `n_particles` times and append those particles to our `particle_cloud`, which is responsible for the rest of the functionality. 
2. `update_robot_pose`
    This function is responsible for updating the robot's pose based on the particles. It normalizes the particles to ensure their weights sum up to 1. Then, it identifies the particle with the highest weight, considering it as the best estimate of the current position. The robot's pose is updated to this best particle's pose. This approach is a key part of the particle filter algorithm, where the estimated robot pose is the one that best aligns with the sensor data.
3. `update_particles_with_odom`
    This function updates the particles based on the robot's odometry data. It calculates the change in position and orientation (delta_x, delta_y, delta_theta) since the last update and applies these changes to each particle. This step is crucial for maintaining the consistency of the particle cloud with the robot's movement, ensuring that the particles follow the trajectory of the robot.
4. `update_particles_with_laser`
    This function updates the particles using laser scan data. It transforms the laser scan data to the map frame for each particle and calculates a weight based on the proximity of the scan points to the nearest obstacles. This step is vital for aligning the particles with the physical environment, as it adjusts their likelihood based on how well they match the observed data.
5. `normalize_particle`
   This function handles the normalization of our particle cloud's weights. 
6. `resample_particle`
   We go into detail about this is our 
   design decision spotlight below.


# Design Decision
One key design decision we had to make was related to our implementation of `resample_particle`, which handles the processing and reproduction of (hopefully) more accurate particles. There are a number of ways to create new points around the points that were the best from the previous sampling.

For the sake of simplicity, our implementation ended up being relatively naive:
    
1. Ensure that the weights of particles form a valid probability distribution via normalization
2. Generate a new set of indices corresponding to particles based on the normalized weights of particles using `draw_random_sample`, which will bias our resampled particles towards higher weights.
3.  Update the particle cloud by copying particles based on the sampled indices.
4.  Add noise (from a normal distribution 0-1) to the particle positions to introduce variability
5.  Renormalize the weights of the newly sampled particles.

# Challenges
1. We had an absurd amount of difficulty just reading C++. Perhaps it's because we are spoiled with Python's lack of typing, but trying to keep track of each datatype (especially what data type was within a list) was really challenging. 

2. Because we had so much trouble learning C++, it was difficult for us to collaborate and evenly divvy up tasks. Our initial idea was to each implement a part of the particle filter, but that meant that large swaths of code were untested when integration began. 

3. We thought that we understood the matrix math that had to be done in order to implement the particle filter, so we kept on debugging the code, thinking it must've just been an implementation error, or us using the helpers wrong... but after going over it with a CA, we realized the fundamental math behind our particle filter was straight up wrong. 

4. C++ makes it hard to test because you *have* to rebuild the code every time. Part of this is that it's annoying to have to do that, but part of it is that we were not in the habit of doing this, so we would consistently forget and wonder why our fixes weren't fixing anything. 


# Improvements
In hindsight, we would've probably finished the project more quickly if we just group-programmed the entire thing. While it's true that we should've taken more care to define the I/Os of our functions, in the end our lack of familiarity made it so we made a ton of small assumptions (about types, or what was publically avilable in each function, etc.) that made integration near-impossible. 

In the end, we basically restarted and tried it again, avoiding all of the mistakes we made the first time.

# Interesting Lessons

This is not a new lesson, but doing things in your brain is hard. This goes for coordinate frames -- if you don't at least list out what coordinate frames need to be considered at every point in time, you'll struggle a lot to figure out why the particles don't *quite* move the way you want it to move -- but it also goes for probabiliy distributions, software architecture, and pretty much anything else. Code is powerful but it is no substitute for a sketch. 

Do not delete the starter code. Even on accident. Just don't do it @Anmol 

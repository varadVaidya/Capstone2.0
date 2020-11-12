# Simulation of Kuka YouBot for Pick and Place Task.
## Capstone Project for the Coursera Specialization [Modern Robotics: Mechanics, Planning, and Control](https://www.coursera.org/specializations/modernrobotics)

This project aims to simulate a mobile manipulator Kuka youBot. youBot is a mobile manipulator with 4 omnidirectional wheels and a 5 DOF robotic manipulator attached to the wheel base.
The mobile base is used to make up for the loss of configuration space, since the Manipulator is only 5 DOF.  
The simulation is done using Python by the help of [Modern Robotics Library](https://github.com/NxRLab/ModernRobotics) and simulated in Coppelia Sim.

## Working
The Trajectory is defined between the inital position of the bot and makes the bot pick the cube and place it at the desired location.
The desired trajectory is a list of End Effector position represented in SE(3).

A FeedForward-PI control then tracks the end effector along the trajctory. The function provides a Spatial Twist in the sapce frame which is then used to calculted to find the required joint and wheel velocites using the Jacobian. These velocities are then in then used to calculate the next state of the youBot.

## Results

## Future Work
* Design an algorithm that will activly avoid the singularity condition and avoid reaching the joint limits.
* Implemet weighted pseudo-Inverse of the Jacobian Matrix to force the youBot to use its mobile base more than its Manipulator
* Implement full state Trajectory Generation and Control.

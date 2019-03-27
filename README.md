# Elliot2
Introducing Elliot2: A new way to program your robot!

With Elliot2, programming autonomous no longer comes with the overhead of having to reposition your robot, having to plug in a microUSB in the middle of your robot, or even having to touch your robot.

Instead, it is now possible to program with the controller! This project aims to make this feasible, with as many features as possible.

## Features
During autonomous, the robot is capable of doing the following motions:
  - Direct Base Control
  - Move to Position
  - Rotate
  - Move in a Straight Line (SLine)
  - Move Mechanism (Intake, Puncher, Angler)
  - Shoot
  - Delay
  - Automatic Ball Tracking w/ Vision Sensor

These, when put together in a list of auton motions and a starting position, form a completed auton. Using the included controller menu, you can edit autonomi without having to re-compile.

These autonomi save to the microSD card on your V5 Brain, as a json file (save.json). 

This program also features odometry using tank kinematics. This makes it possible to move to an absolute position during autonomous, regardless of physical obstruction, like caps. 

## catOS
The catOS menu system is <s>a confusing mess</s> a glorious and minimalistic UI for robot configuration.

To get started with catOS, press the <- and -> buttons together. Use the up/down arrows to view the entries in a list, and the A/B buttons to navigate in and out of menus.

The autonomous & auton editor lists are special in that they have an extra menu to create/move/duplicate/delete items. To access this menu, press X. Most of the options will not prompt you for anything, but they have happened.

Data will be saved when exiting any list editor. This now includes MotionEditor, which means an auton will save much more frequently.

##Documentation
Documentation for this project can be found at [https://ungato.tk/botdocs/](https://ungato.tk/botdocs/).

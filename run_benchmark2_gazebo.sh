#!/bin/bash

TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &

ros2 launch small_robot_gazebo just_measure.launch.py

pkill SIGINT ros2
pkill SIGINT ros2

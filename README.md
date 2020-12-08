# ROS Navigation Turtulebot3 Burger
**Version 1.0.0**

Authors:
- Azadeh Hadadi (a.hadadi1363@gmail.com)
- Jue Wang (834185446@qq.com)

**Table-of-contents**
- [Project Description](#projectDEscription)
- [Hardware Component](#hardwareComponent)
- [Navigation](#navigation)
- [Procedure](#procedure)
- [Discussion](#discussion)
  - [Mapping](#mapping)
  - [Localization](#localization)
  - [Path Planning ](#pathPlanning )
  - [Avoid Obstacles](#avoidObstacles)
  - [Follow Waypoints](#FollowWaypoints)
  
  ## Project Description

This project addressed the problem of ROS Navigation. Turtlebot3 Burger is considered as the objective robot model. A cafeteria map is defined as a target map which is shown in Figure1.
A clean map of the full cafeteria should be created and the robot should localize itself in the environment. A goal position will be defined in the map which the robot should be taken through from its initial position to it. As is displayed in Figure1, there are some obstacles in the map. Meanwhile path planning, Turtulebot3 should avoid these obstacles. As the last step, three waypoints are considered in the map as displayed in Figure2 which Turtulebot should follow.

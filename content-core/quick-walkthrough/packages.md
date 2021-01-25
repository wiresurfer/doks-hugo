---
title: "Packages"
description:
type: dev-tutorials
date: 2018-11-23T10:47:03+05:30
pre: "2. "
weight: 102
---
## Turtle package
The Turtle package emulates the behavior of a Turtlebot through the Simulator.
There may be multiple instances (deployments) of Turtle component, each
representing a single robot entity. In a real-world implementation,
the Turtle package would directly control a robot. Since this is a simulation,
the dynamics of the Turtle's behavior is obtained from the Simulator.

![Turtle package](/images/tutorials/turtlesim/packages/package_turtle.png?classes=border,shadow)

| Name | ROS Interface Type | Description |
| ---- | ------------------ | ----------- |
| /pose | Topic (Pub.) | Turtle's pose, i.e. position and orientation with respect to a predefined inertial reference frame. |
| /sim/cmd_vel | Topic (Pub.) | Velocity commands intended for the Simulator. |
| /turtle_0/goto_action, /action_topics | Action | Commands the Turtle to autonomously go to a position in the environment. |
| /turtle_1/goto_action, /action_topics | Action | Commands the Turtle to autonomously go to a position in the environment. |

--------------------------------------------

| Name | ROS Interface Type | Description |
| ---- | ------------------ | ----------- |
| /cmd_vel | Topic (Sub.) | Velocity commands, which control the desired motion of the Turtle. |
| /sim/pose | Topic (Sub.) | Robot's pose estimates that are published by the Simulator. |
| /sim/sensors | Topic (Sub.) | Simulated robot sensor measurements that are published by the Simulator. The sensors simulated here provide range measurements, which are used to avoid bumping into other Turtles or into the environment boundary. |
| /register_turtle | Service | Uniquely identifies and registers the Turtle. |
| /register_sim_turtle | Service | Uniquely identifies and registers the Turtle with the Simulator. |


## User Interface package
The User Interface is an interactive, web-based visualization of the original
ROS Turtlesim. You can visualize the state of the Turtles, pass control commands to the Turtles. A docker container encapsulates its application code and dependencies. It communicates with the Command Center through a WebSecureSocket. It is the only non-ROS package based on Nginix.

![User Interface package](/images/tutorials/turtlesim/packages/package_user_interface.png?classes=border,shadow)

## Simulator package
The Simulator package replicates the physical dynamics of 2D robots and
their sensors. It publishes two ROS topics - /sim/pose and /sim/sensors.
It subscribes to the ROS topic /sim/cmd_vel that the Turtle package publishes.
It supports two ROS services - /teleport_turtle and /register_sim_turtle.

![Simulator package](/images/tutorials/turtlesim/packages/package_simulator.png?classes=border,shadow)

| Name | ROS Interface Type | Description |
| ---- | ------------------ | ----------- |
| /sim/pose | Topic (Pub.) | Robot's pose, that is, position and orientation relative to an inertial coordinate frame. |
| /sim/sensors | Topic (Pub.) | Simulated robot sensor measurements. The sensors used provide range measurements to any other object in the environment and/or its boundaries. |
| /register_sim_turtle | Service | Used to uniquely identify and register the Simulator with the Turtle. |
| /teleport_turtle | Service | Used to arbitrarily position and orient the robot anywhere in the simulated environment. |

----------------------------------

| Name | ROS Interface Type | Description |
| ---- | ------------------ | ----------- |
| /sim/cmd_vel | Topic (Sub.) | Velocity commands, which are sent from the Turtles. |


## Command Center package
The Command Center is a node for registering Turtles, and a router for
passing control commands and telemetry messages between User Interface
and Turtles.

![Command Center package](/images/tutorials/turtlesim/packages/package_command_center.png?classes=border,shadow)

| Name | ROS Interface Type | Description |
| ---- | ------------------ | ----------- |
| /cmd_vel | Topic (Pub.) | Velocity commands, which control the desired motion of the Turtle. |
| /register_turtle | Service | Uniquely identifies and registers the Turtle. |

----------------------------------

| Name | ROS Interface Type | Description |
| ---- | ------------------ | ----------- |
| /pose | Topic (Sub.) | Turtle's pose, i.e., position and orientation relative to a predefined inertial reference frame. |
| /teleport_turtle | Service | Used to arbitrarily position and orient the robot anywhere in the simulated environment. |
| /turtle_0/goto_action, /action_topics | Action | Commands the Turtle to autonomously go to a position in the environment. |
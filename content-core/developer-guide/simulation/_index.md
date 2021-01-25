---
title: "Simulation"
description:
type: developer-guide
date: 2019-10-30T15:55:25+05:30
pre: "VI. "
weight: 575
---
> A robotics simulator is a simulator used to create application for a physical robot without depending on the actual machine, thus saving cost and time. In some case, these applications can be transferred onto the physical robot (or rebuilt) without modifications. — [Wikipedia](https://en.wikipedia.org/wiki/Robotics_simulator)

During application development a robotics simulator can greatly help by cutting down the cost and time associated with managing real robots. This effect is more pronounced during initial phases of development where more code and configuration iterations happen with higher rate of trial-and-error phases.

[Gazebo](http://gazebosim.org) is popular open-source 3D robotics simulator that has been around for more than a decade, and is used by robotics developers around the world. It has an active [community](http://answers.gazebosim.org/questions) and plethora of [documentation](http://gazebosim.org/tutorials) that helps new users get on-board easily. Gazebo team maintains quite a few 3D [models](https://bitbucket.org/osrf/gazebo_models/) that are readily available for use in projects. The team also maintains [ROS packages](http://wiki.ros.org/gazebo_ros_pkgs) for applications that want to interact with Gazebo through ROS interfaces (topics, services, and actions).

rapyuta.io lets users run their simulation application with Gazebo simulator on the cloud. It provides a web interface to watch and interact with Gazebo GUI client running on the cloud. ROS interfaces provided by its ROS packages complements rapyuta.io’s [ROS communication support](/developer-guide/manage-software-cycle/communication-topologies/ros-support/), and enhances the ways users architect their simulation and robotics applications. For instance, robotics application **running on a device** can interact with Gazebo **running on cloud** through ROS interfaces (e.g. to get model state, set physics properties). This requires, however, that launch files for simulation are not tightly coupled with the rest of the application so they can be run independently.

{{% notice info %}}
The following sample-walkthrough demonstrates how to create such a setup keeping in mind that it should still run on local machine besides rapyuta.io: [Separating Simulation and Application](/build-solutions/sample-walkthroughs/separate-navigation-simulation/).
{{% /notice %}}

{{% notice note %}}
As of now rapyuta.io supports **Gazebo version 9**, and only for executables with [Catkin recipe](/developer-guide/create-software-packages/builds/#catkin-recipe). Users are expected to use gazebo9 ROS packages for their corresponding ROS distros. For instance, `ros-melodic-gazebo-ros-pkgs` on ROS melodic, and `ros-kinetic-gazebo9-ros-pkgs` on ROS kinetic.
{{% /notice %}}


### Using robot_description and other parameters for both Simulation and Application

One limitation while running the above setup is that the ROS parameter server is not shared. This means, for instance, *robot_description* loaded by robotics application on device (for use by [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)) will not be available to Gazebo simulation running on cloud (to spawn URDF model). Since such parameters that need to be shared are not modified after being set, both simulation launch files and application launch files can set them for their respective ROS masters. [Controller](http://wiki.ros.org/ros_control#Controllers) parameters are another example besides *robot_description*.


### Accounting for network delays while communicating over the internet
When [communicating over the public internet](/developer-guide/manage-software-cycle/communication-topologies/ros-support/#ros-over-the-public-internet) through ROS interfaces, communication can incur significant delays. The delays may or may not affect simulation and robotics applications, depending on how much tolerant application is towards these delays. The following ways can be used to account for such delays:

- Have low [QoS](/developer-guide/create-software-packages/ros-support/#defining-qos) for high-frequency topics like [/clock](http://gazebosim.org/tutorials/?tut=ros_comm#GazeboPublishedTopics). It is the default value when enabling simulation for an executable.

- Run Gazebo at a slower speed by reducing real-time factors (which in turn reduces */clock* frequency). This can be done by modifying *max_step_size* and *real_time_update_rate* to decrease the real-time factor. In gzclient, these are updated by selecting *Physics* in the *World* tab. Or, in SDF world file, they can be updated in *\<physics>* tag. The following changes in SDF world file will reduce the real-time factor from 1 (the default) to 0.5, thus running Gazebo simulation at half the normal speed:

```xml
<physics type='ode'>
  <real_time_update_rate>100</real_time_update_rate>  <!-- default: 1000   -->
  <max_step_size>0.005</max_step_size>                <!-- default: 0.001  -->
  <real_time_factor>0.5</real_time_factor>            <!-- default: 1      -->
</physics>
```

- Use */gazebo/pub_clock_frequency* parameter to manually set Gazebo's /clock publish frequency. Keep in mind this does not slow down the simulation and only lowers /clock’s frequency, which might not be ideal for time-sensitive applications. By default, it’s 1000Hz. It can be set to a lower value in a simulation launch file, like so:

```xml
<param name="/gazebo/pub_clock_frequency" value="500" />
```

- If a ROS package provides parameters to modify tolerance for message delays, then they might help too. For instance, [costmap_2d](http://wiki.ros.org/costmap_2d) has a parameter *transform_tolerance* for message delays.

If none of this helps and sharing ROS master is the only way, then simulation and the application must be run in the same Package [Component](/developer-guide/create-software-packages/package-internals/#components).


### gazebo9 ROS kinetic packages on Ubuntu 16.04 (xenial)

rapyuta.io only supports Gazebo version 9. When using Gazebo ROS packages, it assumes those to be gazebo9. When working on Ubuntu xenial, specifying Gazebo ROS dependencies in *package.xml* will fetch gazebo7 ROS packages. For instance, `<depend>gazebo_ros</depend>` in *package.xml* will fetch `ros-kinetic-gazebo-ros`, which is a package for gazebo7. To install gazebo9 packages, the user will need to add the rosdep source-list from https://github.com/osrf/osrf-rosdep/blob/master/gazebo9/00-gazebo9.list. On the local machine execute in terminal:

```bash
wget -qP /etc/ros/rosdep/sources.list.d/ https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo9/00-gazebo9.list
rosdep update
```

After this, installing `<depend>gazebo_ros</depend>` should fetch you `ros-kinetic-gazebo9-ros`. While building your simulation application, this is what rapyuta.io does too.


{{% notice info %}}
Follow the sample walkthrough: [Basic Simulation](/build-solutions/sample-walkthroughs/basic-simulation/)
{{% /notice %}}
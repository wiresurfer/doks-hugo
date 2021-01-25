---
title: "Separating Simulation and Application"
description:
type: build-solutions
date: 2019-11-20T14:46:28+05:30
pre: "9. "
weight: 651
---
## Learning Objective
The walkthrough demonstrates a navigation application that controls
turtlebot3 in a Gazebo simulation on rapyuta.io

It shows the separation of launch files into application and simulation.
It is recommended to run the simulation on the cloud and the
application on either a computer or an instance of the cloud.

## Prerequisites

1. The CPU architecture of the device is **AMD64**
2. Install **ROS Melodic** on the device.
3. Install **catkin-tools** package on the device.
4. Install the [Chrome web browser](https://www.google.com/chrome/) on a computer.
5. Familiarity with tools like git, UNIX/LINUX command terminal.
6. ***Optional***: ROS launch system concepts and ROS navigation stack structure are suggested readings if you want to run your applications with Gazebo on rapyuta.io. It will help you understand the usage better.

## Difficulty
Intermediate

## Estimated Time
25 minutes

## Background
The source code for the walkthrough is in the
[turtlebot_navigation](https://github.com/rapyuta-robotics/io_simulation_tutorials/tree/master/turtlebot_navigation) repository on GitHub.

The simulation and navigation application start from separate
launch files so that the former runs on the cloud and the
latter runs on a device.

The package, ***io_gazebo_turtlebot_bringup***, includes files
that start a demo application for navigating a turtlebot3 model
in a Gazebo simulation. These files are:

* **common.launch**    
  * load configuration parameters shared between
    simulation and navigation application to respective ROS parameter
    servers, for example, *robot_description*, an initial position
    of the TurtleBot.
* **app.launch**    
  * launches amcl, move_base and other navigation related nodes through ***io_gazebo_turtlebot_navigation.launch***
  * launches demo application that sends sequential move_base goals through ***demo_app.launch***
  * loads common configuration parameters via ***common.launch***
* **sim.launch**     
  * loads Gazebo simulation of turtlebot3 via
  ***io_gazebo_turtlebot_gazebo.launch*** file.
  * loads common configuration parameters via ***common.launch***
* **bringup.launch**    
  * launches both ***sim.launch*** and ***app.launch***.
  * ensures ***common.launch*** is called only once.

## Prepare Device
On the device's command line terminal, execute the following commands
in sequence to set up a catkin workspace, install ROS dependencies
and build the catkin workspace.

```bash
cd $HOME
```

```bash
mkdir -p catkin_ws/src
```

```bash
cd catkin_ws/src
```

```bash
git clone https://github.com/rapyuta-robotics/io_simulation_tutorials.git
```

```bash
rosdep init
```

```bash
rosdep update
```

```bash
rosdep install --from-paths --ignore-src io_simulation_tutorials/turtlebot_navigation/ -y --rosdistro melodic
```

```bash
cd ../
```

```bash
source /opt/ros/melodic/setup.bash
```

```bash
catkin build io_gazebo_turtlebot_bringup io_gazebo_turtlebot_demo_app io_gazebo_turtlebot_description io_gazebo_turtlebot_navigation
```

```bash
source devel/setup.bash
```

## Onboard Device

1. On the left navigation bar, click **DEVICES**.
2. Click **ADD NEW DEVICE**.
3. The name of the device is `Turtlebot3 Navigation Simulation Device`.
4. Provide the absolute path of the catkin workspace in the **ROS Catkin Workspace** box. In this case, the workspace is `/home/rapyuta/catkin_ws`. The absolute path of your catkin workspace may be different, which can be determined by executing command ***pwd*** inside the root of the catkin workspace on the device's terminal. 
5. Define the purpose of the device in the **Description** box.
6. Click **CONTINUE**.
7. Click **COPY** to copy the generated device token.
8. Execute the token at the device's terminal to set up rapyuta.io's device agent on the device.

## Create build
To create the build, follow below steps: 

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `io-simulation-navigation` 
4. In the **Git repository** box, enter the url address : 
`https://github.com/rapyuta-robotics/io_simulation_tutorials` and select **Build Recipe** as Catkin.
5. In the context directory, enter `turtlebot_navigation`
6. Click on next, select **ROS Version** as **Melodic** and select the **Has Simulation** option.  
7. Click on next, the build will be created.

The build takes about two to five minutes to build the source code in the ***io_simulation_tutorials***
repository into a running docker container. You may analyse the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which help debug failing builds.

Please proceed to creation of package once the build is Complete.

## Create Packages
You will create and add two packages, namely, Navigation Application and Turtlebot3 Simulation.

#### Navigation Application Package

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. The name of the package: `Navigation Application`
4. Make sure **Is a singleton package** is ***not selected***.
5. Ensure **Is a bindable package** is ***selected***.
6. The version of the package is `1.0.0`
7. The purpose of the package is to `Controlled navigation of turtlebot3`
8. Click **NEXT**.
9. The name of the component: `navigation_component`
10. The runtime of the component is **Device**.
11. Ensure **Is ROS Component** is selected.
12. Choose **Melodic** for **ROS Version**.
13. Set **Restart Policy** to **Never**.
14. The name of the executable: `navigation_executable`
15. The **Executable Type** is **Default**.
16. In the **Command to run in the docker container** box, enter the command: `roslaunch io_gazebo_turtlebot_bringup app.launch`
17. Click on **Add ROS topic**. The name of the ROS topic is `/cmd_vel`, and it is set **QoS** to **Low**.
18. Add a configuration parameter by clicking on **Add Parameter**. The name of the parameter is `SPAWN_TURTLEBOT_ROBOT`. The **Default** value is `true`.
19.  Click **NEXT** > **CONFIRM PACKAGE CREATION**.

#### Simulation Package

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. The name of the package: `Simulation`
4. Make sure **Is a singleton package** is ***not selected***.
5. Ensure **Is a bindable package** is ***selected***.
6. The version of the package is `1.0.0`
7. The purpose of the package is to `Simulation of turtlebot3`
8. Click **NEXT**.
9. The name of the component: `simulation_component`
10. The runtime of the component is **Cloud**.
11. Ensure **Is ROS Component** is selected.
12. Choose **Melodic** for **ROS Version**.
13. The number of **Replicas to run the component** is **1**
14. The name of the executable: `simulation_executable`
15. For **Executable Type**, click on **Builds**.
16. In the **Choose Build** select the Build (`io-simulation-navigation`) [created above](/build-solutions/sample-walkthroughs/separate-navigation-simulation/#create-build)
	from the drop-down list
17. In the **Command to run in the docker container** box, enter the command: `roslaunch io_gazebo_turtlebot_bringup sim.launch gui:=true`
18. Set **Resource Limit** to **Medium:2 cpu cores, 8 GiB memory**
{{% notice warning %}}
For simulation, the resource limit should either be **Medium** or **Large**. Simulation has issues with **Small** resource limits.
{{% /notice %}}
19. Add the following **ROS topics**:
    1.  **Name**: `/joint_states`, **QoS**: Low
    2.  **Name**: `/tf`, **QoS**: Low
    3.  **Name**: `/scan`, **QoS**: Low
    4.  **Name**: `/odom`, **QoS**: Low
20. Click **NEXT**.
21. Under **Inbound ROS Interfaces**, Click on **Add Topic** to add the ROS topic `/cmd_vel` as an inbound ROS topic.
22. Click **CONFIRM PACKAGE CREATION**.

## Deploy Packages

You will first deploy the Simulation package, and then the Navigation Application package.

#### Deploy Simulation Package

1. On the left navigation bar, click **CATALOG**.
2. Select **Simulation** package.
3. Click **Deploy package**.
4. The name of deployment: `SIMULATION`
5. Enter the value for **VNC_PASSWORD**
6. Click **CREATE DEPLOYMENT** > **Confirm**

You will redirect to the **Details** tab of the newly
created deployment. The **SIMULATION** is successfully running
if the progress bar reaches **Succeeded**, and the status is **Running**.

#### Deploy Navigation Application Package

1. On the left navigation bar, click **CATALOG**.
2. Select **Navigation Application** package.
3. Click **Deploy package**.
4. The name of deployment: `NAVIGATION`
5. Select **Turtlebot3 Navigation Simulation Device** as the device on which the **navigation_component** will be deployed.
6. Check if the **SPAWN_TURTLEBOT_ROBOT** parameter has the value *true*.
7. Check if the **ros_wokspace** and **ros_distro** device configuration variables have values set to the correct absolute path for *catkin_ws* and *melodic* respectively.
8. Click **Add dependency** to add **SIMULATION** deployment as a dependent deployment.
9. Click **CREATE DEPLOYMENT** > **Confirm**

You can verify if **NAVIGATION APPLICATION** is running successfully by
checking if the progress bar reaches **Succeeded** and status is
**Running**.

## Result

1. On the **Details** tab of **SIMULATION** deployment, copy the value of the network endpoint ***vnc***.
2. Paste the copied URL address in the address bar of the web browser and press Enter.
3. Enter the value of ***VNC_PASSWORD***, which you provided while deploying the package when prompted.

<video controls style="max-width: 1500px" width="100%" class="border shadow" src="/images/tutorials/separate-navigation-simulation/expected-outcome.webm"></video>

## Advanced Tips
If you want to run your application separate from the Gazebo simulation on
rapyuta.io, it will need time synchronization. Since the set
up runs two ROS Masters, one in the cloud with Gazebo and the other on a
computer with navigation nodes. The clock for these two applications will need
to be in sync. The clock synchronization is by ***/clock*** ROS topic, which is published by
Gazebo.

* [***/use_sim_time*** is set to ***true***](https://github.com/rapyuta-robotics/io_simulation_tutorials/blob/e50ccd9b9d8a99af8d4ae15b361d1370443bd2aa/turtlebot_navigation/io_gazebo_turtlebot_bringup/config/common_config.yaml#L5) in ROS parameter server
  to use ***/clock*** topic
* If your application requires time synchronization, you have to wait on
  the ***/clock*** topic as shown [here](https://github.com/rapyuta-robotics/io_simulation_tutorials/blob/e50ccd9b9d8a99af8d4ae15b361d1370443bd2aa/turtlebot_navigation/io_gazebo_turtlebot_demo_app/scripts/demo_app.py#L13).



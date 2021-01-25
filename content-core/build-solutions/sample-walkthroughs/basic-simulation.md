---
title: "Basic Simulation With Teleoperation"
description:
type: build-solutions
date: 2019-10-24T13:48:07+05:30
pre: "8. "
weight: 650
---
## Learning Objective
The tutorial will show you how to simulate a robot in a 3D world using rapyuta.io

## Prerequisites

1. The CPU architecture of the device is AMD64.
2. The version of ROS running on the device is ROS Melodic.
3. Install the Google Chrome browser.
4. Be familiar with git, UNIX/LINUX command terminal.

## Difficulty
Intermediate

## Estimated Time
30 minutes

## Tutorial Walkthrough
In this tutorial, you will first add a device to rapyuta.io.
You will run turtlebot3 simulation on the cloud, and then run
keyboard teleoperation from the device.

## Add a Device

1. On the left navigation bar, click **DEVICES**.
2. Click **ADD NEW DEVICE**.
3. The name of the device is `Teleop Device`
4. Select **Use docker compose as default runtime** option.
5. Select **Melodic** as **ROS Version**.
6. In the **Description** box, type in `demonstrate communication to Gazebo simulation running on the cloud.`
7. Click **CONTINUE**.
8. Click **COPY** to copy the generated token.
9. Run the copied token in the device's terminal. The token sets up rapyuta.io device agent on the device.

When the device is successfully registered, you will see a green dot next to the device's name, indicating that the device is online.

## Create builds
You will create two builds in the tutorial.
### Simulation build
To create the build, follow below steps:

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `io-simulation-teleoperation` 
4. In the **Git repository** box, enter the url address : 
`https://github.com/rapyuta-robotics/io_simulation_tutorials` and select **Build Recipe** as Catkin.
5. In the context directory, enter `turtlebot_teleoperation`
6. Click on next, select **ROS Version** as **Melodic** and select the **Has Simulation** option.  
7. Click on **Add Parameter** under **CATKIN BUILD PARAMETERS**,
    1. add the following **ROS Packages**:`turtlebot3_gazebo turtlebot3_description`
    2. add the following packages to **Blacklist**:
    `turtlebot3 turtlebot3_bringup turtlebot3_example turtlebot3_navigation turtlebot3_slam`	
8. Click on next, the build will be created.

The build takes about two to five minutes to build the source code in the ***io_simulation_tutorials***
repository into a running docker container. You may analyse the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which help debug failing builds.


### Teleoperation build
To create the build, follow below steps : 

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `turtlebot-teleoperation` 
4. In the **Git repository** box, enter the url address : 
`https://github.com/ROBOTIS-GIT/turtlebot3#melodic-devel` and select **Build Recipe** as Catkin.
5. Click on next, select **Architecture** as amd64 and select **ROS Version** as **Melodic**.  
6. Click on **Add Parameter** under **CATKIN BUILD PARAMETERS**,
    1. add the following **ROS Packages**:`turtlebot3_teleop`
    2. add the following packages to **Blacklist**:`turtlebot3 turtlebot3_bringup turtlebot3_example turtlebot3_navigation turtlebot3_slam turtlebot3_description`	
7. Click on next, the build will be created.

The build takes about two to five minutes to build the source code in the ***turtlebot3***
repository into a running docker container. You may analyse the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which help debug failing builds.

Please proceed to creation of package once the builds are Complete.

## Create Packages
You will create two packages in the tutorial.
### Turtlebot3 Robot Simulation Package

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. The name of the package is `Turtlebot3 Robot Simulation`.
4. Make sure **Is a singleton package** is ***not selected***.
5. Ensure **Is a bindable package** is ***selected***.
6. The version of the package is `1.0.0`
7. The purpose of the package is to `run Gazebo simulation on the cloud`
8. Click **NEXT**.
9. The name of the component is `Simulator`.
10. The runtime of the component is **Cloud**.
11. Ensure **Is ROS Component** is selected.
12. Choose **Melodic** for **ROS Version**.
13. The number of replicas to run the component is `1`
14. The name of the executable is `turtlebot3_autotrace_launcher`
15. For **Executable Type**, click on **Builds**.
16. In the **Choose Build** select the Build (`io-simulation-teleoperation`) [created above](/build-solutions/sample-walkthroughs/basic-simulation/#simulation-build)
	from the drop-down list
17. In the **Command to run in the docker container** box, enter the command: `roslaunch turtlebot3_gazebo turtlebot3_autorace.launch`
18. Set **Resource Limit** to **Medium:2 cpu cores, 8 GiB memory**.
{{% notice warning %}}
For simulation, the resource limit should either be **Medium** or **Large**. Simulation has issues with **Small** resource limits.
{{% /notice %}}
19. Click **Add Topic** under **Inbound ROS Interfaces** to add the ROS topic a deployment of this package subscribes to for velocity commands. The name of the topic is `/cmd_vel`
20. Click **CONFIRM PACKAGE CREATION**. 


### Turtlebot3 Keyboard Teleoperation Package

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. The name of the package is `Turtlebot3 Keyboard Teleoperation`
4. The version number of the package is `1.0.0`
5. Make sure **Is a singleton package** is ***not selected***.
6. Ensure **Is a bindable package** is ***selected***.
7. The purpose of the package is to `control the robot using the keyboard.`
8. The name of the component is `teleop`
9. The runtime of the component is **Device**.
10. Ensure **Is ROS Component** is selected.
11. Select **Melodic** for **ROS Version**.
12. The CPU architecture is **amd64**.
13. Set **Restart Policy** to **Always**.
14. The name of the executable is `sleep`
15. For **Executable Type**, click on **Builds**.
16. In the **Choose Build** select the Build (`turtlebot-teleoperation`) [created above](/build-solutions/sample-walkthroughs/basic-simulation/#teleoperation-build)
	from the drop-down list
17. In the **Command to run in the docker container** box, enter the command: `sleep infinity`
{{% notice info %}}
The command, ***sleep infinity***, will keep the deployment running and does not serve any other purpose. 
{{% /notice %}}
18. Add a ROS topic by clicking on **Add ROS topic**. The name of the topic is `/cmd_vel`, and its **QoS** is set to **Low**.
19. Click **NEXT** > **CONFIRM PACKAGE CREATION**.


## Deploy Packages
You will deploy all the packages created in the previous step.

### Robot Simulation

1. On the left navigation bar, click **CATALOG**.
2. Select **Turtlebot3 Robot Simulation** package.
3. Click **Deploy Package**.
4. The name of the deployment is `ROBOT SIMULATION`
5. Enter the value for **VNC_PASSWORD**.
6. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the **Details** tab of the newly created deployment. The **ROBOT SIMULATION** is successfully running if the progress bar reaches **Succeeded**, and the status is **Running**.

### Keyboard Teleoperation

1. On the left navigation bar, click **CATALOG**.
2. Select **Turtlebot3 Keyboard Teleoperation** package.
3. Click **Deploy Package**.
4. The name of the deployment is `KEYBOARD TELEOPERATION`
5. Select **Teleop Device** to deploy the ***teleop*** component.
6. Add **ROBOT SIMULATION** as a dependent deployment by clicking **Add dependency**.
7. Click **CREATE DEPLOYMENT** > **Confirm**.

You can verify if **KEYBOARD TELEOPERATION** is running successfully by checking if the progress bar reaches **Succeeded** and status is **Running**.

## Simulation

1. On the **Details** tab of **ROBOT SIMULATION** deployment, copy the value of the network endpoint ***vnc***.
2. Paste the copied URL address in the address bar of the web browser and press Enter.
3. Enter the value of ***VNC_PASSWORD***, which you provided while deploying the package, when prompted. You will view the 3D world and the **Teleop Device** (robot) inside it.
4. Go to the **Shell Access** tab of **KEYBOARD TELEOPERATION** deployment, and click on **SSH**.
5. Enter the following commands in sequence:
   1. `bash`
   2. `. /opt/catkin_ws/devel/setup.bash`
   3. `TURTLEBOT3_MODEL=burger roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
6. Try controlling the robot using the keyboard commands shown in ther terminal output. To see how the robot moves, switch to the Gazebo simulation browser tab.
7. To reset simulation, **Ctrl+C** and then run `rosservice call /gazebo/reset_simulation`

<video controls style="max-width: 1500px" width="100%" class="border shadow" src="/images/tutorials/turtlebot-simulation/simulation.webm"></video>

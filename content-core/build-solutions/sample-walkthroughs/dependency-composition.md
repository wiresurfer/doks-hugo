---
title: "Dependency Composition"
description:
type: build-solutions
date: 2019-10-24T13:47:35+05:30
pre: "5. "
weight: 635
---
A deployment may depend on other deployments. It can access all
topics, services, actions, endpoints and configuration parameters exposed by
other deployments at runtime.

## Learning Objectives
The tutorial will show you how to create a dependent deployment using
[rapyuta.io console](https://console.rapyuta.io).

## Prerequisites

1. Device requirements
	1. You should have access to a device (computer or Raspberry PI 2 or 3)
	with an internet connection.
	2. Install the [Google Chrome](https://www.google.com/chrome) browser is
	on the device.
	3. Ensure that the [ROS Kinetic Kame](https://wiki.ros.org/kinetic/Installation) is installed on the device.
{{% notice note %}}
If the device has [ROS Melodic Morenia](http://wiki.ros.org/melodic)
installed on it, replace ***Kinetic*** with ***Melodic*** in all places
where a specific version of ROS is asked for. The tutorial should still
work the same.
{{% /notice %}}
1. You should be familiar with the [map_server](https://wiki.ros.org/map_server) ROS package.
2. You should be familiar with the below tools:
	1. [Git](https://git-scm.com/doc)
	2. UNIX/LINUX [command terminal](https://www.digitalocean.com/community/tutorials/an-introduction-to-the-linux-terminal)
	3. [ROS topics](https://wiki.ros.org/Topics)
	4. [ROS nodes](https://wiki.ros.org/Nodes)
	5. [ROS services](https://wiki.ros.org/Services)

## Difficulty
Intermediate

## Estimated Time
30 minutes

## Tutorial Walkthrough
You will add and deploy the *dynamic_map_server* package. It is a modified version of the original *map_server* package. The package offers a
navigation map to other deployments that depend on it. Besides exposing the ROS topics: */map* and */map_metadata*, the package also exposes the */set_map* service,
which replaces the map published by */map* topic.

#### Creating the build
To create the build, follow below steps. Skip the following steps if you have already created an *io-tutorials* build earlier.

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `io-tutorials` 
4. In the **Git repository** box, enter the url address : 
`https://github.com/rapyuta/io_tutorials` and select **Build Recipe** as Catkin.
5. Go to the next step and click on next, the build will be created.

The build takes about two to five minutes to build the source code in the *io_tutorials*
repository into a running docker container. You may analyse the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which help debug failing builds.

Please proceed to creation of package once the build is Complete.

#### Create dynamic_map_server package
1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. You should provide information about the package, such as the name of the
package, its version, whether it is a singleton package, and a short description.
   1. In the **Package Name** box, type in the name of the package say
	   `dynamic_map_server`
   2. In the **Package Version** box, type in the version of the package. By
	   default, the version is set to _1.0.0_
   3. Ensure **Is a singleton package** option is ***not selected***.
   4. Make sure **Is a bindable package** option ***is selected***.
   5. In the **Description** box, provide a brief summary of the package, for
	   example, `A modified ROS map_server`
   6. Click **NEXT**
4. In the **Component Name** box, enter a name for the component say
   `DynamicMapServer`
{{% notice info %}}     
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and underscore _ character, and must not begin with a digit.
{{% /notice %}}
5. Select **Cloud** for **Component Runtime**.
6. Ensure **Is ROS Component** is selected.
7. Set the value of **Replicas to run the component** to number 1 (default value).
8. In the **Executable Name** box, type in a name for the executable say
   `dmsexecutable`  
{{% notice info %}} 
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and underscore _ character, and must not begin with a digit.
{{% /notice %}}
9. For **Executable Type**, click on **Builds**.
10. In the **Choose Build** select the Build (`io-tutorials`) [created above](/build-solutions/sample-walkthroughs/dependency-composition/#creating-the-build)
	from the drop-down list
11. In the **Command to run in the docker container** box, copy and paste the command:
	```bash
	roslaunch dynamic_map_server map_server.launch
	```
	
	Ensure you always execute the *roslaunch* command for explicitly starting the
	[ROS Master](http://wiki.ros.org/Master) instead of running the *rosrun*
	command, because the ROS Master will fail to start on _rosrun_, and
	eventually, the deployment will fail as well.

	![Executable details](/images/tutorials/dms/dms-exec-details.png?classes=border,shadow&width=50pc)
12. To add a ROS topic, click **Add ROS topic**. In the **Name** box,
    enter `/map_metadata` and set **QoS** to **Low**.
    Similarly, add another ROS topic `/map` and set **QoS** to **Low**.
		![Add ROS topic](/images/tutorials/dms/dms-add-ros-topics.png?classes=border,shadow&width=50pc)
13. To add a ROS service, click **Add ROS service**. In the **Name** box, enter
    `/set_map`
		![Add ROS service](/images/tutorials/dms/dms-add-ros-service.png?classes=border,shadow&width=50pc)
14. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

Additionally, you may verify if the package is built successfully and is ready
to be deployed by clicking to see if the **Deploy package** button is enabled.

#### Prepare and add a device
You will [prepare](/developer-guide/manage-machines/special-device-tutorials/#preparing-raspberry-pi-3) a Raspberry Pi 2 or 3
as the device for this tutorial.

If you are using custom rapyuta.io image on the device, the catkin workspace is already created, and the *io_tutorials* repository is already present in the workspace. Moreover, the source code is built for you.

{{% notice note %}}
In this tutorial, the catkin workspace is `~/catkin_ws/`, but you may choose to name your catkin workspace as you like and ensure that you replace all occurrences to `~/catkin_ws/` with your workspace name.
{{% /notice %}}

If you are using either a computer with ROS installed on it or any device other
than Raspberry PI, or a Raspberry PI without custom rapyuta.io image, you will
create a catkin workspace and get the *io_tutorials* repository into the workspace.

Hence, to create a catkin workspace on the device, you have to execute the following
commands at the device terminal.
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
git clone https://github.com/rapyuta/io_tutorials
```

```bash
source /opt/ros/kinetic/setup.bash
```

```bash
cd ..
```

For the custom rapyuta.io image to support the build command, ***catkin build***, you will set up the device by executing the following:
```bash
cd $HOME && 
mv catkin_ws catkin_old && 
curl https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm32v7.tar.gz | tar xz
``` 

The argument to the ***curl*** command, i.e., the URL address, changes based on the architecture of the device.

* For a device with an *arm64* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm64v8.tar.gz
* For a device with an *arm32* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm32v7.tar.gz
* For a device with an *amd64* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_amd64.tar.gz

To build the source code in the catkin workspace, execute the below command in the root of
the workspace:
```bash
catkin build map_listener
```
{{% notice note %}}
If you experience the error ***catkin:command not found***, then the *python-catkin-tools* package is missing on the device, which is required for executing *catkin build* command. Install the package by running the command `sudo apt-get install python-catkin-tools` at the device terminal.
{{% /notice %}}

And then, you will [add the device](/developer-guide/manage-machines/onboarding/) to rapyuta.io.
{{% notice note %}}
While adding the device, ensure that **Use docker compose as default runtime** checkbox is ***not selected***.
{{% /notice %}}

#### Create map_listener package
You will create *map_listener* package, which will be deployed on the device.
To create the package, follow the instructions:

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You should provide information about the package such as the name of the
   package, its version number, whether it's a singleton package and a description.
	1. In the **Package Name** box, enter a name for the package say `map_listener`
	2. In the **Package Version** box, enter the version of the package. By default,
		the version is set to _1.0.0_
	3. Ensure **Is singleton package** option is ***not selected***.
	4. Make sure **Is a bindable package** option ***is selected***.
	4. In the **Description** box, provide a summary of the package say
	   `Runs a map_listener node on device`
	5. Click **NEXT**.
3. In the **Component Name** box, provide a name for the component say `MapListener`
4. For **Component Runtime**, click **Device**.
5. Ensure **Is ROS Component** is selected.
6. Ensure the **ROS Version** is **Kinetic**.
7. In the **Executable Name** box, enter a name for the executable say
   `map_listener_executable`
8. For **Executable Type**, click **Default**.
9. In the **Command to run on the device** box, copy and paste the command:
	```bash
	roslaunch map_listener listener.launch
	```

	Ensure you always execute the *roslaunch* command to explicitly start the [ROS Master](http://wiki.ros.org/Master) instead of running the *rosrun* command, because the ROS Master will fail to start on *rosrun* command, and
	eventually, the deployment will fail as well.
	![map_listener_executable](/images/tutorials/dms/maplistener_exec_details.png?classes=border,shadow&width=50pc)
9. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

#### Deploy dynamic_map_server package
To deploy *dynamic_map_server* package, follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Select *dynamic_map_server* package.
3. Click **Deploy package**.
4. In the **Name of deployment** box, provide a name for the specific deployment
   you are creating say `Dynamic Map Server Deployment`
5. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** page.
The _Dynamic Map Server Deployment_ is successfully running only when the green
colored progress bar moves to **Succeeded** and **Status:Running**, indicating that the **DEPLOYMENT PHASE** is **Succeeded**, and the **STATUS** is **Running**.

![Dynamic Map Server Deployment](/images/tutorials/dms/dms-deployment.png?classes=border,shadow&width=50pc)

#### Deploy map_listener package
To deploy *map_listener* package, follow the steps:

1. Click **CATALOG** > select *map_listener* package > click **Deploy package**.
2. In the **Name of deployment** box, provide a name for the specific deployment
   say `Map Listener Deployment`
3. Since *map_listener_executable* has device runtime, you must select the device you want to deploy the component on. Click **Refresh the list of online devices** to retrieve an updated list of online devices.
4. Select the device from the **Select device for deploying the component** drop-down list.
5. Ensure that the **ros_workspace** and **ros_distro** are selected.
   ![Deploy map_listener package](/images/tutorials/dms/deploy-mplstnr.png?classes=border,shadow&width=40pc)
6. Click **Add dependency** to add a dependent deployment.
   ![Add dependent deployment](/images/tutorials/dms/add-dependency.png?classes=border,shadow&width=40pc)
7. Select _Dynamic Map Server Deployment_ from the drop-down list of deployments.
   Ensure that the _Dynamic Map Server Deployment_ is valid and is already running.
8. Click **CREATE DEPLOYMENT** > **Confirm**.

You can verify if the _Map Listener Deployment_ is successfully running by
checking if the green colored progress bar indicates that the **DEPLOYMENT PHASE** is _Succeeded_ and the **STATUS** is _Running_.

![Map Listener Deployment](/images/tutorials/dms/mplstnr-deployment.png?classes=border,shadow&width=60pc)

Ensure that the dependent deployment **STATUS** is _Running_ as well.

The corresponding [dependency graph](/developer-guide/manage-software-cycle/compose-software/dependency-graph/) will look as shown below:
![Dependency graph](/images/tutorials/dms/dms-dgraph.png?classes=border,shadow&width=50pc)

To know whether *map_listener* has received the map data, execute the below
command in the device's terminal:

```bash
sudo tail /root/.ros/log/latest/map_listener-2.log
```

{{% notice info %}}
Sometimes *map_listener* stores the map data in *map_listener-1.log* file. Therefore, you are recommended to check all the files of the form
***map_listener-n.log*** where **_n_** is a positive integer if any file is empty.
{{% /notice %}}

You should see a similar output as shown below after executing the above command:

```bash
[rosout][INFO] 2018-01-26 06:18:56,565: Received map data
[rosout][INFO] 2018-01-26 06:18:56,578: Read a 4000 X 4000 map @ 0.0500000007451 m/cell
```

#### Update navigation map
In the device's terminal window, execute the command:

```bash
sudo tail -f /root/.ros/log/latest/rosout.log
```

Open another terminal window, and run the command:

```bash
source ~/catkin_ws/devel/setup.bash
```
To pass an argument to the service ***/set_map***, press the tab key (more than twice) to complete the ***rosservice call*** command:
```bash
rosservice call /set_map "map:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  info:
    map_load_time: {secs: 0, nsecs: 0}
    resolution: 0.0
    width: 0
    height: 0
    origin:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  data: [0]
initial_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

In the previous terminal window, you should see that the *map_listener* will
receive a new map. The result may appear as shown below:
```bash
... INFO [listener.py:7(callback) [topics: /rosout, /map] Received map data
... INFO [listener.py:11(callback) [topics: /rosout, /map] Read a 0 X 0 map @ 0.0 m/cell
```
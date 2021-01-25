---
title: "Preinstalled Runtime"
description:
type: build-solutions
date: 2019-10-24T13:47:04+05:30
pre: "a. "
weight: 620
---
A _ROS publisher_ is part of a ROS package. It is a public git repository, which is built into a running docker container on the fly when the package
is being deployed. A _ROS subscriber_ is also a part of the same ROS package. It is downloaded on a device and is launched when the package is deployed.

## Learning objectives
The tutorial will show you how to deploy a basic ROS package with a _ROS publisher_ running on the cloud and a _ROS subscriber_ running on a device such as Raspberry PI.

## Prerequisites
1. Device requirements
	1. You should have access to a device (computer or Raspberry PI 2 or 3)
	with an internet connection.
	2. Ensure that the [Google Chrome](https://www.google.com/chrome) browser is installed on the computer.
	3. Ensure that the [ROS Kinetic Kame](https://wiki.ros.org/kinetic/Installation) is installed on the device.
{{% notice note %}}
If the device has [ROS Melodic Morenia](http://wiki.ros.org/melodic)
installed on it, replace ***Kinetic*** with ***Melodic*** at all places
where a specific version of ROS is asked for. The tutorial should still
work the same.
{{% /notice %}}
2. You should read the [developer guide](/developer-guide/) of rapyuta.io
3. You should be familiar with the below tools:
	1. [Git](https://git-scm.com/doc)
	2. UNIX/LINUX [command terminal](https://www.digitalocean.com/community/tutorials/an-introduction-to-the-linux-terminal)
	3. [ROS topics](https://wiki.ros.org/Topics)
	4. [ROS services](https://wiki.ros.org/Services)

## Difficulty
Beginner

## Estimated time
15 minutes

## Preparing your device
The tutorial will use Raspberry PI as the device.
Learn [how to prepare your Raspberry PI](/developer-guide/manage-machines/special-device-tutorials/#preparing-raspberry-pi-3)

If you are using the custom rapyuta.io image on the device, the catkin workspace is already created for you, and the *io_tutorials* repository
is already present in the workspace. Moreover, the source code is
already built for you.

{{% notice note %}}
In this tutorial, the catkin workspace is `~/catkin_ws/`, but you
may choose to name your catkin workspace as you like and ensure
that you replace all occurrences to `~/catkin_ws/` with your
workspace name.
{{% /notice %}}

If your device is a computer with ROS installed on it, or a Raspberry PI
without custom rapyuta.io image, you will create a catkin workspace and
get the *io_tutorials* repository into that workspace.

Hence, to create a catkin workspace, you have to execute the below commands at the device terminal.
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
```
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

To build the source code in the catkin workspace, execute the below
command in the root of the workspace:
```bash
catkin build listener
```
{{% notice note %}}
If you experience an error ***catkin:command not found***, then the *python-catkin-tools* package is missing on the device, which is required for executing *catkin build* command. Install the package by running `sudo apt-get install python-catkin-tools` at the terminal.
{{% /notice %}}

## Setting up your device
To onboard the device on to rapyuta.io,[add the device](/developer-guide/manage-machines/onboarding/) to the console. Ensure that you do not select
the **Use docker compose as default runtime** checkbox while adding the device.

## Creating the build
To create the build, follow below steps. Skip the following steps if you have already created an *io-tutorials* build earlier.

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `io-tutorials` 
4. In the **Git repository** box, enter the url address : 
`https://github.com/rapyuta/io_tutorials` and select Build Recipe as Catkin.
5. Go to the next step and click on next, the build will be created.

The build takes about two to five minutes to build the source code in the *io_tutorials* repository into a running docker container. You may analyze the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which helps in debugging failed builds.
Please proceed to creation of package once the build is Complete.


## Creating the package
To create a package using the [console](https://console.rapyuta.io), follow
the steps:

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. In the **Package Name** box, type in a name for the package say `ROS publisher subscriber`.
4. In the **Package Version** box, enter the version of the package you are creating.
   The default value is _1.0.0_
5. Make sure **Is singleton package** is not selected.
6. Ensure **Is a bindable package** is selected.
7. In the **Description** box, provide a summary of the package.
8. Click **NEXT**.

The package has two components: the **talker** running on the cloud and the
**listener** running on the device.

1. Talker component (aka _ROS publisher_)
	1. In the **Component Name** box, enter a name for the component say `talker`
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and an underscore _ character. It must not begin with a digit.
{{% /notice %}}
	2. For **Component Runtime**, click **Cloud**.
	3. Ensure **Is ROS Component** is selected.
	4. Set the value of **Replicas to run the component** number 1 (default value).
	5. In the **Executable Name** box, enter a name for the executable say `talkerExecutable`   
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits[0-9], hyphen - and an underscore _ character, and must not start with a digit.
{{% /notice %}}
	6. For **Executable Type**, click on **Builds**.
	7. In the **Choose Build** select the Build (`io-tutorials`) [created above](/build-solutions/sample-walkthroughs/basic-ros-pubsub/preinstalled-runtime/#creating-the-build)
	from the drop-down list.	
	8. In the **Command to run in the docker container** box, enter the command:
	   	```bash
	   	roslaunch talker talker.launch
	   	```

	   	Ensure you always execute the command *roslaunch* to explicitly start the
	   	[ROS Master](https://wiki.ros.org/Master) instead of running the *rosrun*
	   	command, because the ROS Master will fail to start on *rosrun*, and
	   	eventually, the deployment will fail as well.
	   ![talkerExecutable](/images/tutorials/ros-pub-sub/ros-pubsub-talker-exec-details.png?classes=border,shadow&width=50pc)
	9. The _talkerExecutable_ publishes a ROS topic, `/telemetry`    
	   To add a ROS topic, click **Add ROS topic**. In the **Name** box, enter the name of the ROS topic. Select **Maximum** for **QoS**.
2. Listener component (aka _ROS subscriber_)
	1. In the **Component Name** box, type in a name for the component, say `listener` 
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and an underscore _ character, and must not begin with a digit.
{{% /notice %}}
	2. For **Component Runtime**, click **Device**.
	3. Ensure **Is ROS Component** is selected.
	4. Ensure the **ROS Version** is **Kinetic**.
	5. In the **Executable Name** box, type in a name for the executable say `listenerExecutable`  
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and an underscore _ character, and must not begin with a digit.
{{% /notice %}}
	6. Since the _ROS subscriber_ is already installed on the device, select
	   **Default** as **Executable Type**.
	7. In the **Command to run in the docker container** box, enter the command:
		```bash
		roslaunch listener listener.launch
	   	```

	   	Ensure you always execute the command *roslaunch* to explicitly start the
	   	[ROS Master](https://wiki.ros.org/Master) instead of running the *rosrun*
	   	command, because the ROS Master will fail to start on *rosrun*, and
	   	eventually, the deployment will fail as well.
	   ![listenerExecutable](/images/tutorials/ros-pub-sub/ros-pubsub-listener-exec.png?classes=border,shadow&width=50pc)
	8. Click **NEXT** > **CONFIRM PACKAGE CREATION**.



## Deploying the package
To deploy a package using the [console](https://console.rapyuta.io),
follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Select the **ROS publisher subscriber** package.
3. Click **Deploy package**.
4. In the **Name of deployment** box, enter a name for the deployment you are
creating say `ROS Publisher Subscriber Deployment`.
5. Since _listener_ has device runtime, you must select the device you want to
deploy the component on. Click **Refresh the list of online devices** to retrieve
an updated list of online devices.
6. Select the device from the **Select device for deploying the component**
drop-down list.
7. For the _listener_ component, ensure that **ros_workspace** and **ros_distro** are selected.
8. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** page. The _ROS Publisher Subscriber Deployment_ is successfully running only when the green colored bar moves to **Succeeded** and **Status:Running** point indicating that the
**DEPLOYMENT PHASE** is **Succeeded**, and the **STATUS** is **Running**.

![ROS  Publisher Subscriber Deployment](/images/tutorials/ros-pub-sub/ros-pub-sub-deployment.png?classes=border,shadow&width=50pc)

You may also analyse the corresponding [deployment logs](/developer-guide/tooling-automation/logging/deployment-logs/)
to check if everything is working OK by clicking on **Logs** tab.

The **listener-listenerExecutable** will be streaming ***/listener I heard hello_world*** logs.

![ROS Subscriber Logs](/images/tutorials/ros-pub-sub/listener-logs.png?classes=border,shadow&width=50pc)

while **talker-talkerExecutable** will be publishing ***hello_world*** logs.
![ROS Publisher Logs](/images/tutorials/ros-pub-sub/talker-logs.png?classes=border,shadow&width=50pc)
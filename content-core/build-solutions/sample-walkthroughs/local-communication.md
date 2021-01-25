---
title: "Local Communication"
description:
type: build-solutions
date: 2019-10-24T13:47:47+05:30
pre: "6. "
weight: 640
---
Complex robotic applications involving multi-device communication can be
latent when the service is distributed across WAN. This tutorial demonstrates
how to have multi-device communication within the same LAN.

## Learning objectives
This tutorial will show you how to deploy a broker package locally for inter
device communication using [rapyuta.io console](https://console.rapyuta.io).

## Prerequisites
1. Device requirements
	1. You should have access to three devices (computer or Raspberry PI 2 or 3)
	   with an internet connection.
	2. Install the latest [Google Chrome](https://www.google.com/chrome/)
	   browser on the device.
	3. Ensure the [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) (ROS kinetic) is installed on the device.
{{% notice note %}}
If the device has [ROS Melodic Morenia](http://wiki.ros.org/melodic)
installed on it, replace ***Kinetic*** with ***Melodic*** in all places
where a specific version of ROS is asked for. The tutorial should still
work the same.
{{% /notice %}}
1. You should be familiar with the following tools:
	1. ROS [topics](http://wiki.ros.org/Topics)
	2. UNIX/LINUX [command terminal](https://www.digitalocean.com/community/tutorials/an-introduction-to-the-linux-terminal)

## Difficulty
Intermediate

## Estimated time
20 minutes

## Tutorial walkthrough

In this tutorial, you will add three devices namely _Broker Device_,
_Publisher Device_ and _Subscriber Device_. You will also create and deploy
_ROS Publisher_ and _ROS Subscriber_ packages.

#### Add Broker Device
The _Broker Device_ must be of **amd64** CPU architecture.

1. Click **DEVICES** > **ADD NEW DEVICE**.
2. In the **Device Name** box, enter the name of the device say `Broker Device`
3. Select **Use docker compose as default runtime** option.
4. Ensure the **ROS Version** is **Kinetic**.
5. In the **Description** box, provide a summary of the device
   say `I am a communication broker`
6. Click **CONTINUE**.
7. Click **COPY** to copy the generated **Token**

Paste and execute the token (otherwise called the device setup link) in the device's terminal to set up the rapyuta.io client on the device.

If the device is set up successfully, you should see the following output at the device's terminal:
```bash
Initialising the Rapyuta Platform

############(100%)
Successfully Installed!
```

Ensure that there's a <span style="color:green">**green**</span> dot next to the ***Broker Device***,
which indicates that it is online on rapyuta.io.

{{% notice note %}}
In production-like scenarios, ensure the [broker device is assigned a static IP address](/developer-guide/manage-software-cycle/communication-topologies/local-communication-broker/).
{{% /notice %}}

#### Prepare Publisher Device
The _Publisher Device_ is:

* Raspberry PI 2 or 3
* can have either **arm64v8** or **arm32v7** CPU architecture
* must have ROS Kinetic installed on it
* must have rapyuta.io tutorials installed on it

{{% notice info %}}
The custom rapyuta.io image comes with [Ubuntu Xenial](http://releases.ubuntu.com/xenial/) OS and
[ROS kinetic](http://wiki.ros.org/kinetic) software installed on them.
Moreover, the [rapyuta.io tutorials](https://github.com/rapyuta-robotics/io_tutorials) are also installed on these custom images.
{{% /notice %}}

{{% notice info%}}
Learn how to [prepare Raspberry PI](/developer-guide/manage-machines/special-device-tutorials/#preparing-raspberry-pi-3)
{{% /notice %}}

If you are using custom rapyuta.io image on the device, the catkin workspace is
already set up for you, the ***io_tutorials*** repository is already downloaded
in the workspace, and the source code is already built for you.

{{% notice note %}}
In this tutorial, the catkin workspace is `~/catkin_ws/`, but you
may choose to name your catkin workspace as you like and ensure
that you replace all occurrences to `~/catkin_ws/` with your
workspace name.
{{% /notice %}}

If you are using either a computer with ROS Kinetic installed on it, or
a Raspberry PI without custom rapyuta.io image, you will create a catkin
workspace and get the ***io_tutorials*** repository into the workspace.

To create the catkin workspace, you have to execute the below commands at the device's terminal.

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

The argument to the ***curl*** command, i.e., the URL address,
changes based on the architecture of the device.

* For a device with an *arm64* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm64v8.tar.gz
* For a device with an *arm32* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm32v7.tar.gz
* For a device with an *amd64* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_amd64.tar.gz

To build the source code in the catkin workspace, execute the below
command in the root of the workspace:
```bash
catkin build talker
```
{{% notice note%}}
If you experience the error ***catkin:command not found***, then the *python-catkin-tools* package is missing on the device, which is required for executing *catkin build* command. Install the package by running the command `sudo apt-get install python-catkin-tools` at the device terminal.
{{% /notice %}}

## Add Publisher Device

1. Click **DEVICES** > **ADD NEW DEVICE**.
2. In the **Device Name** box, enter the name of the device say `Publisher Device`
3. In the **ROS Catkin Workspace** box, enter the absolute path of the catkin
   workspace found on the device.
   If rapyuta.io custom image is installed on the device, the absolute path
   of the catkin workspace is `/home/rapyuta/catkin_ws`.    
   Otherwise, the absolute path of the catkin workspace will be different
   for the device.
4. In the **Description** box, provide a summary of the device say
   `I am a ROS Publisher`
5. Click **CONTINUE**.
6. Click **COPY** to copy the generated **Token**.

Paste and execute the token in the device's terminal to set up the
rapyuta.io client on the device.

If the device is set up successfully, you should see the following output
at the device's terminal:
```bash
Initialising the Rapyuta Platform

############(100%)
Successfully Installed!
```

Ensure that there's a <span style="color:green">**green**</span> dot next to
the ***Publisher Device***, which indicates that it is online on rapyuta.io.

## Prepare Subscriber Device
The _Subscriber Device_ is a:

* Raspberry PI 2 or 3
* can have either **arm64v8** or **arm32v7** CPU architecture
* must have ROS Kinetic installed on it
* must have rapyuta.io tutorials installed on it

{{% notice info %}}
The custom rapyuta.io image comes with [Ubuntu Xenial](http://releases.ubuntu.com/xenial/)
OS and [ROS Kinetic](http://wiki.ros.org/kinetic) software installed on them.
Moreover, the [rapyuta.io tutorials](https://github.com/rapyuta-robotics/io_tutorials)
are also installed on these custom images.
{{% /notice %}}

{{% notice info %}}
Learn how to [prepare Raspberry PI](/developer-guide/manage-machines/special-device-tutorials/#preparing-raspberry-pi-3)
{{% /notice %}}

If you are using custom rapyuta.io image on the device, the catkin workspace is
set up for you, the ***io_tutorials*** repository is downloaded in the workspace, and the source code is built for you.

{{% notice note %}}
In this tutorial, the catkin workspace is `~/catkin_ws/`, but you
may choose to name your catkin workspace as you like and ensure
that you replace all occurrences to `~/catkin_ws/` with your
workspace name.
{{% /notice %}}

If you are using either a computer with ROS installed on it or a
Raspberry PI without custom rapyuta.io image, you will create a
catkin workspace and get the ***io_tutorials*** repository into
the workspace.

To create the catkin workspace, you have to execute the below commands
at the device's terminal:

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

The argument to the ***curl*** command, i.e., the URL address,
changes based on the architecture of the device.

* For a device with an *arm64* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm64v8.tar.gz
* For a device with an *arm32* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_arm32v7.tar.gz
* For a device with an *amd64* architecture, use https://storage.googleapis.com/artifacts.rapyuta.io/io_tutorials/catkin_ws_amd64.tar.gz

To build the source code in the catkin workspace, execute the
below command in the root of the workspace:
```bash
catkin build listener
```
{{% notice note%}}
If you experience the error ***catkin:command not found***, then the *python-catkin-tools* package is missing on the device, which is required for executing *catkin build* command. Install the package by running the command `sudo apt-get install python-catkin-tools` at the device terminal.
{{% /notice %}}

## Add Subscriber Device

1. Click **DEVICES** > **ADD NEW DEVICE**.
2. In the **Device Name** box, enter the name of the device say `Subscriber Device`
3. In the **ROS Catkin Workspace** box, enter the absolute path of the
   catkin workspace found on the device.    
   If rapyuta.io custom image is installed on the device, the absolute path
   of the catkin workspace is `/home/rapyuta/catkin_ws`    
   Otherwise, the absolute path of the catkin workspace will be
   different for the device.
4. In the **Description** box, provide a short summary of the device say
   `I am a ROS Subscriber`
5. Click **CONTINUE**.
6. Click **COPY** to copy the generated **Token**.

Paste and execute the token in the device's terminal to set up the device
manager client on the device.

If the device is set up successfully, you should see the following output:
```bash
Initialising the Rapyuta Platform

############(100%)
Successfully Installed!
```

Ensure that there's a <span style="color:green">**green**</span> dot next to
the ***Subscriber Device***, which indicates that it is online on rapyuta.io.

## Create ROS Publisher package

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You should provide information about the package such as the name of the
   package, its version, whether it is a singleton package, and a
   description.
   1. In the **Package Name** box, enter the name of the package say `ROS Publisher`
   2. In the **Package Version** box, type in the package's version. By default,
      the version is set to _1.0.0_
   3. Ensure **Is a singleton package** checkbox is ***not selected***.
   4. Ensure **Is a bindable package** checkbox is ***not selected***.
   5. In the **Description** box, provide a summary of the package say
      `Publishes ROS topic for a subscriber`
   6. Click **NEXT**.
3. In the **Component Name** box, enter a name for the component, say `Publisher`
4. Select **Device** as the **Component Runtime**.
5. Ensure **Is ROS Component** is selected.
6. Ensure the **ROS Version** is **Kinetic**.
7. In the **Executable Name** box, type in a name for the executable, say `talker`
8. For **Executable type**, select **Default** because the source code is already installed on the _Publisher Device_.
9. In the **Command to run in the docker container** box, copy and paste the command:
   	```bash
   	roslaunch talker talker.launch
   	```

	Ensure you always execute *roslaunch* command for explicitly starting the
    [ROS Master](http://wiki.ros.org/Master) instead of running the *rosrun* command, because the ROS Master will fail to start on _rosrun_, and eventually, the deployment will fail as well.
9. To add a ROS topic, click **Add ROS topic**. In the **Name** box, enter `/telemetry`
   and set **QoS** to **Maximum**.
10. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

## Create ROS Subscriber package

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You should provide information about the package such as the name of the
   package, its version, whether it is a singleton package, and a
   short description.
   1. In the **Package Name** box, enter the name of the package say `ROS Subscriber`
	2. In the **Package Version** box, type in the package's version. By default,
	   the version is set to _1.0.0_
	3. Ensure **Is a singleton package** checkbox is ***not selected***.
	4. Ensure **Is a bindable package** checkbox is ***not selected***.
	5. In the **Description** box, provide a summary of the package say
	   `Subscribes to ROS topic published by a publisher`
	6. Click **NEXT**.
3. In the **Component Name** box, enter a name for the component, say `Subscriber`
4. Select **Device** as the **Component Runtime**.
5. Ensure **Is ROS Component** is selected.
6. Ensure the **ROS Version** is **Kinetic**.
7. In the **Executable Name** box, type in a name for the executable, say `listener`
8. For **Executable type**, select **Default** because the source code is already installed on the _Subcriber Device_.
9. In the **Command to run in the docker container** box, copy and paste the command:
	```bash
	roslaunch listener listener.launch
	```

	Ensure you always execute *roslaunch* command for explicitly starting the
	[ROS Master](http://wiki.ros.org/Master) instead of running the *rosrun*
	command, because the ROS Master will fail to start on _rosrun_, and
	eventually, the deployment will fail as well.
9. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

## Deploy local communication broker

1. Click **CATALOG**.
2. Under **Communication packages**, select **Rapyuta IO Local Communication Broker** package.
3. Click **Deploy package**.
4. In the **Name of Deployment** box, enter a name for the broker deployment say `Communication Broker Deployment`
5. Since **brokerComponent** has **Device runtime** select the device you want to deploy on by clicking **Refresh the list of online devices**. It retrieves
an updated list of online devices.
{{% notice note %}}
As the component will be deployed on a device, its restart policy is already set to **Always**, however, you may override this value at the time of creating a deployment of the package.
{{% /notice %}}
6. Select **Broker Device** from **Select device for deploying the component** drop-down list.
7. Select the network interface parameter value as per your device on
   which you are deploying by clicking **NETWORK_INTERFACE** drop-down list.
8. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** tab.
The package is successfully deployed when the green colored bar moves
from **In progress** to **Succeeded** indicating that the **DEPLOYMENT PHASE**
has **Succeeded**, and the **STATUS** is **Running**.

You may analyse the corresponding [deployment logs](/developer-guide/tooling-automation/logging/deployment-logs/)
so you may debug if the deployment fails.

## Deploy ROS Publisher package

1. Click **CATALOG** > select **ROS Publisher** package > click **Deploy package**.
2. In the **Name of deployment** box, enter a name for the deployment
   say `ROS Publisher Deployment`
3. Since **Publisher** has **Device runtime**, select the device you want to deploy on by clicking **Refresh the list of online devices**. It retrieves an
updated list of online devices.
4. Select **Publisher Device** from **Select device for deploying the component** drop-down list.
5. Under **Device Config Variables**, ensure that the **ros_workspace** and
   **ros_distro** are selected.
6. Click **Add dependency** to add a dependent deployment.
7. Select **Communication Broker Deployment** from the drop-down list of
   deployments. Ensure that the **Communication Broker Deployment** is
   valid and is already running.
   ![Dependent deployment](/images/tutorials/local-comm-broker/ros-pub-dependent-deploy.png?classes=border,shadow&width=50pc)
8. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the **Details** tab of the newly created deployment. The package is successfully deployed when the green coloured bar moves from
**In progress** to **Succeeded** indicating that the **DEPLOYMENT PHASE** has **Succeeded**
and the **STATUS** is **Running**.

![ROS Publisher Deployment](/images/tutorials/local-comm-broker/ros-pub-deployment.png?classes=border,shadow&width=50pc)

Ensure that the dependent deployment **STATUS** is **Running** as well.

You may analyse the corresponding [deployment logs](/developer-guide/tooling-automation/logging/deployment-logs/) so you may debug
if the deployment fails.

The corresponding [dependency graph](/developer-guide/manage-software-cycle/compose-software/dependency-graph/) of **ROS Publisher Deployment** looks like:
![Dependency graph](/images/tutorials/local-comm-broker/dgraph-pub-broker.png?classes=border,shadow&width=50pc)

## Deploy ROS Subscriber package

1. Click **CATALOG** > select **ROS Subscriber** package > click **Deploy package**.
2. In the **Name of deployment** box, enter a name for the deployment say
   `ROS Subscriber Deployment`
3. Since **Subscriber** has **Device runtime**, select the device you want to deploy on by clicking **Refresh the list of online devices**. This retrieves an
updated list of online devices.
4. Select **Subscriber Device** from the **Select device for deploying the component** drop-down list.
5. Ensure that **ros_workspace** and **ros_distro** are selected.
6. Click **Add dependency** to add a dependent deployment.
7. Select **Communication Broker Deployment** from the drop-down list of
   deployments. Ensure that the **Communication Broker Deployment** is valid
   and is already running.
8. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** tab.
The package is successfully deployed when the green colored bar moves from
**In progress** to **Succeeded** indicating that the **DEPLOYMENT PHASE** has **Succeeded**, and the **STATUS** is **Running**.

![ROS Subscriber Deployment](/images/tutorials/local-comm-broker/ros-sub-deployment.png?classes=border,shadow&width=50pc)

Ensure that the dependent deployment **STATUS** is **Running** as well.

You may analyse the corresponding [deployment logs](/developer-guide/tooling-automation/logging/deployment-logs/) so you may debug
if the deployment fails.

The corresponding [dependency graph](/developer-guide/manage-software-cycle/compose-software/dependency-graph/) of **ROS Subscriber Deployment** looks like:
![Dependency graph](/images/tutorials/local-comm-broker/dgraph-sub-broker.png?classes=border,shadow&width=50pc)

If all of the above three deployments are successfully running, the
logs of **ROS Subscriber Deployment** will print ***hello_world***.


Since the communication broker is deployed on the **Broker Device** locally,
and the bindable attribute is not selected (value is set to false) for both
the **ROS Publisher** package and the **ROS Subscriber** package, the ROS topic
(***/telemetry***) and in general, the data is transferred within the same
local network. Thus, the application's latency is comparatively reduced
provided the **Broker Device** is in the same network as the **Publisher Device**, and **Subscriber Device**.
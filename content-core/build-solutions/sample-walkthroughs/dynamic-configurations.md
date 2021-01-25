---
title: "Dynamic Configurations"
description:
type: build-solutions
date: 2019-10-24T13:47:57+05:30
pre: "7. "
weight: 645
---
The *Publisher for Configured Devices* is a ROS package that will be deployed on a device, which is configured based on a set of parameters like *name*, *location*, *velocity*, etc.

## Learning objective
The tutorial will show you how to apply configuration parameters to a device before deploying a rapyuta.io package on it.

## Prerequisites
1. Device requirements
   1. You should have access to a device (computer or Raspberry PI 2 or 3) with an internet connection.
   2. Ensure that the [Google Chrome](https://www.google.com/chrome) browser is installed on the computer.
   3. Ensure that the [ROS Kinetic Kame](https://wiki.ros.org/kinetic/Installation) is installed on the device.
{{% notice note %}}
If the device has [ROS Melodic Morenia](http://wiki.ros.org/melodic)
installed on it, replace ***Kinetic*** with ***Melodic*** in all places
where a specific version of ROS is asked for. The tutorial should still
work the same.
{{% /notice %}}
2. You should be familiar with the [configurations](/developer-guide/manage-software-cycle/dynamic-configurations/) concept.
3. Read the quick starting guide on [how to apply configurations to devices](/developer-guide/manage-software-cycle/dynamic-configurations/apply-dynamic-configs/).
4. You should be familiar with the following tools:
   1. [Git](https://git-scm.com/doc)
   2. [UNIX/Linux command terminal](https://www.digitalocean.com/community/tutorials/an-introduction-to-the-linux-terminal)

## Difficulty
Beginner

## Estimated time
15 minutes

## Tutorial walkthrough
The tutorial consists of the below steps:

1. Define a configuration
2. Prepare a device
3. Add the device to rapyuta.io
4. Apply configuration parameters to the device
5. Add a ROS package to rapyuta.io
6. Deploy the package on the device
7. Verify deployment logs

#### Defining configuration
You will define the configuration ***robots*** as follows:

1. On the left navigation bar, click **CONFIGURATIONS**.
2. Click **ADD NEW CONFIGURATION**.
3. Provide a name, say, ***robots*** for the configuration in the **Configuration Name** box.
4. Click **CONFIRM**.
5. Add a new file, ***name.yaml***, below the root node ***robots***.
6. Define parameters in the ***name.yaml***.
{{% notice info %}}
Refer to the quick starting guide on [how to define parameters](/developer-guide/manage-software-cycle/dynamic-configurations/apply-dynamic-configs/#creating-configuration-parameters).
{{% /notice %}}
![robots configuration](/images/tutorials/talker-supervisor/robots-configuration.png?classes=border,shadow&width=40pc)

Similarly, define parameters in all of the ***name.yaml*** files occurring below the various value nodes and the root node. For example,

* **robots** root node
![robots parameter file](/images/tutorials/talker-supervisor/robots-name-file.png?classes=border,shadow&width=40pc)
* **drone** value node
![drone parameter file](/images/tutorials/talker-supervisor/drone-name-file.png?classes=border,shadow&width=40pc)
* **drone-1** value node
![drone-1 parameter file](/images/tutorials/talker-supervisor/drone1-name-file.png?classes=border,shadow&width=40pc)

#### Preparing device
The tutorial will use Raspberry PI as the device. Learn [how to prepare the device](/developer-guide/manage-machines/special-device-tutorials/#preparing-raspberry-pi-3).

If you are using the custom rapyuta.io image on the device, you need to execute the following command to update the ***io_tutorials*** repository at the root of your catkin workspace.

```bash
git pull https://github.com/rapyuta/io_tutorials
```

To build the package, run the below command at the root of your catkin workspace.

```bash
catkin build param_talker
```

#### Adding device to rapyuta.io
When [onboarding the device to rapyuta.io](/developer-guide/manage-machines/onboarding/) the environment variable **RIO_CONFIGS_DIR** is set locally on the device. It is the location of the directory where all of the configurations that will be applied to the device are stored.
{{% notice info %}}
The value of **RIO_CONFIGS_DIR** is set to **/opt/rapyuta/configs**
{{% /notice %}}
![configuration environment variable](/images/tutorials/talker-supervisor/environment-config-var.png?classes=border,shadow&width=40pc)

{{% notice info %}}
**RIO_CONFIGS_DIR** is available to all the executables of a running deployment.
{{% /notice %}}

#### Applying configuration to device
You should [define labels for the device](/developer-guide/manage-software-cycle/dynamic-configurations/device-labels/) so you can apply configuration parameters to it. You will define the following labels in this tutorial:

* robot_type: drone
* robot_name: drone-1

![device labels](/images/tutorials/talker-supervisor/device-21-labels.png?classes=border,shadow&width=40pc) 

To apply an existing configuration to the device:

1. On the left navigation bar, click **DEVICES**.
2. Select the device you would want to apply the configuration to.
3. Click **Apply Configuration Parameters**.
   ![apply configuration parameters](/images/tutorials/talker-supervisor/apply-config-params.png?classes=border,shadow&width=60pc)
4. Click **CONFIRM APPLY**.
   ![confirm configuration parameters application](/images/tutorials/talker-supervisor/confirm-config-params-application.png?classes=border,shadow&width=30pc)

The set of device labels (*robot_type: drone*, *robot_name: drone-1*) are resolved to select the **drone-1/name.yaml** file, and thus, apply the below configuration parameters:
```yaml
name:
    first_name: Drone 1.0
    last_name: RDrone
```
{{% notice note %}}
Had you given a device label say *robot_type: drone*, rapyuta.io would traverse the configuration tree, select the **drone/name.yaml** file and apply the configuration parameters found in the file. Similarly, if no device label is defined, **name.yaml** file will be selected for the application of configuration parameters defined in the file.
{{% /notice %}}
{{% notice info %}}
You may apply more than one configuration to a single device.
{{% /notice %}}

The robots configuration is stored in **RIO_CONFIGS_DIR** and its parameters file ***name.yaml*** is stored in the **robots** (corresponding configuration) directory as shown in the figure below.
![robots configuration directory](/images/tutorials/talker-supervisor/content-rio-configs-dir.png?classes=border,shadow&width=40pc)

![configuration parameters file](/images/tutorials/talker-supervisor/parameter-file.png?classes=border,shadow&width=40pc)

You can use **RIO_CONFIGS_DIR** in ROS launch files for loading configurations.
![configuration environment variable in ROS launch file](/images/tutorials/talker-supervisor/launch-file-content.png?classes=border,shadow&width=50pc)

You can remotely access **RIO_CONFIGS_DIR** by [SSH-ing into the device via rapyuta.io](/developer-guide/tooling-automation/#remote-web-terminal-webssh)
![SSH access](/images/tutorials/talker-supervisor/SSH-into-device.png?classes=border,shadow&width=40pc)

#### Creating the package
To create the *Publisher for Configured Devices* package, follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. Provide a name for the package say `Publisher for Configured Devices` in the **Package Name** box.
4. Make sure **Is singleton package** is ***not selected***.
5. Ensure **Is a bindable package** is ***indeed selected***.
6. In the **Description** box, provide a summary of the package, say, `Demo package to illustrate configuration parameters concept`
7. Click **NEXT**.
8. In the **Component Name** box, enter a name for the component say `parameter_talker`.
9. Select **Device** for **Component Runtime**.
10. Ensure **Is ROS Component** is selected.
11. Ensure the **ROS Version** is **Kinetic**.
12. In the **Executable Name** box, enter a name for the executable say `talker_executable`.
13. Select **Default** for **Executable Type**.
14. Enter the following command in the **Command to run in the docker container** box.
```
roslaunch param_talker talker.launch
```
14. Click **NEXT** > **CONFIRM PACKAGE CREATION**.
![publisher for configured devices](/images/tutorials/talker-supervisor/pub-configured-devices.png?classes=border,shadow&width=40pc)

## Deploying the package
To deploy the *Publisher for Configured Devices* package, follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Select the **Publisher for Configured Devices** package.
3. Click **Deploy package**.
4. Provide a name for the deployment you are creating say `Configurations Publisher` in the **Name of deployment** box.
5. Since *parameter_talker* has device runtime, you must select the device you want to deploy the component on. Click **Refresh the list of online devices** to retrieve an updated list of online devices.
6. Select the device from the **Select device for deploying the component** drop-down list.
7. For the **parameter_talker** component, ensure that **ros_workspace** and **ros_distro** are selected.
8. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** page. The **Configurations Publisher** is successfully running when the green colored bar moves to **Succeeded** and **Status:Running** point indicating that the **DEPLOYMENT PHASE** is **Succeeded** and the **STATUS** is **Running**.

![Configurations Publisher](/images/tutorials/talker-supervisor/talker-supervisor-deployment.png?classes=border,shadow&width=40pc)

## Verifying deployment logs
You may verify the correctness of the tutorial by analyzing the corresponding deployment logs by clicking on the **Historical Logs**.

The historical logs will display the output as shown:

```bash
hello Drone 1.0 RDrone
```

The output contains values of the *first_name* and *last_name* parameters for the *drone-1* device.
![Successful deployment logs](/images/tutorials/talker-supervisor/successful-logs.png?classes=border,shadow&width=60pc)

{{% notice note %}}
Your output may be different based on the device labels you define.
{{% /notice %}}
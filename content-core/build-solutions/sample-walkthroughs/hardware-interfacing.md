---
title: "Hardware Interfacing"
description:
type: build-solutions
date: 2019-10-24T13:47:25+05:30
pre: "4. "
weight: 630
---
The tutorial will teach you how to deploy a basic non-ROS package that will change
the trigger of the on-board LED (ACT/LED0) on Raspberry PI 2 or 3.

## Learning objectives
The goal is to learn how to deploy a basic non-ROS package on a
Raspberry PI 2 or 3 with docker compose runtime.

## Prerequisites
1. Device requirements
	* You should have access to a computer with an internet connection.
	* Ensure that the [Google Chrome](https://www.google.com/chrome/) browser
	  is installed on the computer.
	* Raspberry PI 2 or 3
2. You should be familiar with
   [Docker image](https://docs.docker.com/v17.09/engine/userguide/storagedriver/imagesandcontainers/)
   concept.

## Difficulty
Beginner

## Estimated time
15 minutes

## Preparing your device
Learn how to [prepare your Raspberry PI](/developer-guide/manage-machines/special-device-tutorials/#preparing-raspberry-pi-3).

## Setting up Raspberry PI
To onboard the device on to rapyuta.io,
[add the device](/developer-guide/manage-machines/onboarding/) to rapyuta.io,
and ensure that you select the **Use docker compose as default runtime** checkbox
while adding the device.

## Creating the package
To create ***led_trigger*** package, follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD NEW PACKAGE**.
3. In the **Package Name** box, enter the name for the package as `led_trigger`
4. In the **Package Version** box, type in the version of the package. By default, it is set to _1.0.0_
5. In the **Description** box, enter a summary of the package.
6. Click **NEXT**.
7. In the **Component Name** box, enter a name for the component say `led_trigger`
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and an underscore _ character, and must not start with a digit.
{{% /notice %}}
8. Select **Device** as **Component Runtime**.
9.  Deselect **Is ROS Component** checkbox.
10. Select **arm32v7** as **Architecture**.
11. In the **Executable Name** box, type in a name for the executable say `led_trigger_executable`
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9], hyphen - and an underscore _ character, and must not start with a digit.
{{% /notice %}}
12. Click **Docker** for **Executable Type** as the executable is a docker image.
13. In the **Docker image** box, specify the docker image: `rrdockerhub/led-trigger-arm32v7`
14. In the **Command to run in the docker container** box, enter the command `led_trigger led0 heartbeat`
15.  Deselect **Run command from bash shell**.
16.  Click **NEXT** > **CONFIRM PACKAGE CREATION**.


## Deploying the package
To deploy a package from the rapyuta.io,
follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Select the ***led_trigger*** package that you just created.
3. Click **Deploy package**.
4. In the **Name of deployment** box, enter a name for the deployment say `LED Trigger Deployment`
5. Since *led_trigger* has device runtime, you must select the device you want to deploy the component on. Click **Refresh the list of online devices** to retrieve an updated list of online devices.
6. Select the device from the **Select device for deploying the component**
   drop-down list.
7. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** page.
The _LED Trigger Deployment_ is successfully running only when the green
colored bar moves to **Succeeded** and **Status:Running** point indicating that the **DEPLOYMENT PHASE** is **Succeeded**, and the **STATUS** is **Running**.

You may also analyze the corresponding
[deployment logs](/developer-guide/tooling-automation/logging/deployment-logs/)
to check if everything is working good.

To verify that everything is working correctly, you should observe the trigger
of the on-board LED(ACT/LED0) on Raspberry PI 2 or 3 switches to a heartbeat.

![Onboard LED](/images/core-concepts/device-management/control-onboard-led.gif?classes=border,shadow&width=30pc)
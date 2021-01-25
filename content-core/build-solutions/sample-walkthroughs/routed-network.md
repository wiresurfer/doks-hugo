---
title: "ROS Routed Network"
description:
type: build-solutions
date: 2019-10-24T13:46:19+05:30
pre: "2. "
weight: 611
---

## Learning Objective
The tutorial will show you how to create a ROS routed networks deployed to cloud or a device. 
It will also explain other actions you can take on a created network.

## Prerequisites

1. Install the [Chrome web browser](https://www.google.com/chrome/) on a computer.
2. You should be familiar with onboarding a device on rapyuta.io
3. Device requirements
	1. **AMD64** CPU architecture.  
	2. An active internet connection.
	3. Onboarded onto rapyuta.io with runtime **Docker Compose**

## Difficulty
Intermediate

## Estimated Time
30 minutes

## Creating Cloud Routed Network 
Follow these steps to create a cloud routed network.

1. On the left navigation bar, click **NETWORKS**.
2. Click **ADD NEW ROUTED NETWORK**.
3. Enter a name for routed network.
4. Select **ROS Distro**, as either Kinetic or Melodic based on ROS version of package components it will be binding to.
5. Select the **Runtime** as **Cloud**.
![goo](/images/tutorials/routed-networks/create-cloud-routed-network.png?classes=border,shadow&width=40pc)
6. Click **CONTINUE**.


## Creating Device Routed Network 
Follow these steps to create a device routed network. Make sure you have a rapyuta.io registered
device with docker runtime and AMD64 architecture available.


1. On the left navigation bar, click **NETWORKS**.
2. Click **ADD NEW ROUTED NETWORK**.
3. Enter a name for routed network.
4. Select **ROS Distro**, as either Kinetic or Melodic based on ROS version of package components it will be binding to.
5. Select the **Runtime** as **Device**.
6. You will see a list of online device with docker runtime and AMD64 architecture in the drop-down list. 
Select the **Device** and it’s **IP Interface**. 
7. Select the [Restart policy](/developer-guide/manage-software-cycle/deployments/#restart-policy).
![goo](/images/tutorials/routed-networks/create-device-routed-network.png?classes=border,shadow&width=40pc)
8. Click **CONTINUE**.

Deploying a routed network is identical to deploying any other package and has identical corresponding phases and errors.
Once the routed network deployment succeeds, other ROS package deployments can bind to it and communicate.
![goo](/images/tutorials/routed-networks/routed-network-details.png?classes=border,shadow&width=40pc)

## Deleting Routed Network

Only network not bound to any running deployments can be deleted.

1. On the left navigation bar, click **NETWORKS**. It will display all the routed networks in a given project.
2. Select the routed network which you want to delete. 
3. Click on **Delete**.
4. Confirm on the routed network deletion message.
{{% notice warning %}}
An attempt to deprovision a network that is currently being used will result in an error.
{{% /notice %}}

{{% notice info %}}
Follow this walkthrough to [bind routed networks](/developer-guide/manage-software-cycle/deployments/#deploying-a-package) to ROS package deployments. 
{{% /notice %}}

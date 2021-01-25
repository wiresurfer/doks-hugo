---
title: "May"
type: release-updates
date: 2020-05-04T16:18:22+05:30
weight: 835
---
## May 4

We have released a hotfix to address a vulnerability in SaltStack(which is internally used by rapyuta.io for managing devices.) 
As a security precaution, we have updated authentication keys for all the devices on-boarded to the platform, if you already have an existing device on rapyuta.io platform, please re-onboard the device to use it. 
To re-onboard your device, follow the below steps:     
1. Please select the device    
2. Click on Token button     
3. Copy the curl command    
4. Execute it on the physical device.     
We have preserved all the labels and metadata associated with a device.

Please reach out to us on support@rapyuta-robotics.com if you have any questions or concerns.

## May 6

We have relased a fix for the [quick walkthrough](/quick-walkthrough/) that was broken due to a change in upstream. Following the steps in the walkthrough should help with getting it up and running. If you want to understand more about the workaround, please refer to this [page](/build-solutions/quirks/rosbridge-compatibility).

## May 13
Welcome to the May 13, 2020 release of the rapyuta.io platform. There
are significant updates in this release that we hope you will like.
This release has many internal bug fixes and improvements.

#### Features
Ability to set ROS services and ROS actions as targeted while creating packages.

#### Notable Fixes

* Fixed the limit on the count of components in a package.
* Fixed the hyperlink of included packages in package details page.

#### Documentation
Read about how to [set ROS services and ROS actions as targeted](/developer-guide/manage-software-cycle/communication-topologies/ros-support/).


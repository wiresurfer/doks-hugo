---
title: "Device Configuration Variables"
description:
type: developer-guide
date: 2019-10-24T11:48:15+05:30
pre: "5. "
weight: 265
---
Device configuration variables are environment variables that allow you to
correctly configure a device on rapyuta.io using the
[console](https://console.rapyuta.io).
They are used when deploying packages on the device.

A list of predefined device configuration variables is available for a
newly added device. They are:

* **ros_distro** specifies the ROS distribution found on the device.
  It is automatically detected when the device is provisioned. It is
  immutable as the device determines its value. It is a mandatory
  configuration variable when deploying ROS packages. It is usually
  assigned ***kinetic*** or ***melodic*** value depending on the version of ROS installed on the device.
* **ros_workspace** is the absolute path of the default catkin workspace on the
device. rapyuta.io automatically sources ROS packages present in this workspace
for package deployment.
* **runtime** specifies whether the device supports *dockercompose* for the package deployment. Otherwise, its value is set as *preinstalled*. It is immutable. You can provide this value while adding a device to rapyuta.io

The **Details** page of each device contains the above pre-defined configuration variables.

To create additional device configuration variables, such as
***ros_package_path***, click **Add Config Variable**.

![Device Configuration Variables](/images/core-concepts/device-management/add-device-config-var.png?classes=border,shadow&width=50pc)

You can override the predefined value of a device configuration variable
while creating a package deployment. For instance, you may change ***ros_workspace*** when deploying a package for a ROS-based device.

You can deselect device configuration variables that you do not need during
deployment. However, remember that ***ros_distro*** is mandatory when deploying a
ROS package.
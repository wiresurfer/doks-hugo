---
title: "Supported Devices"
description:
type: developer-guide
date: 2019-10-24T11:47:03+05:30
pre: "2. "
weight: 235
---
The following is the list of requirements for registering devices
on rapyuta.io

* Mandatory
    * Python >=2.7.8, <3
    * [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) (Xenial Xerus)
 or [Ubuntu 18.04](http://releases.ubuntu.com/18.04/) (Bionic Beaver)
* Optional
    * [curl](https://curl.haxx.se/)
    * Robot Operating System (ROS)
      * [Kinetic Kame](http://wiki.ros.org/kinetic) or [Melodic Morenia](http://wiki.ros.org/melodic)

Ubuntu(16.04 and 18.04) by default resolves the hostname to localhost. If you do change this behaviour on the host OS, roscore **will** not be able to start. A simple way to check if roscore can be started is to do `nslookup $(hostname)` if it returns a DNS record you are probably good to go.

{{% notice note%}}
Ensure you install **ros-kinetic-ros-base**, or **ros-melodic-ros-base**, or above it.
{{% /notice %}}

{{% notice info %}}
Read about the device [onboarding process](/developer-guide/manage-machines/onboarding/), which consists of registering, setting up and viewing a device on rapyuta.io
{{% /notice %}}
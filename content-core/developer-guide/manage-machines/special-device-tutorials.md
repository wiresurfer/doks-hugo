---
title: "Special Device Tutorials"
description:
type: developer-guide
date: 2019-10-29T14:32:13+05:30
pre: "4. "
weight: 262
---
Consider the following devices.

* [Generic x64](#generic-x64) - UP Board
* [ARM](#arm) - raspberry-pi

### Generic x64
#### Preparing UP Board
##### Prerequisites
- [UP board](https://up-board.org/up/specifications/)
- DC power supply rated 5V/4A
- USB flash drive
- Monitor screen
- HDMI cable
- USB keyboard
- USB mouse
- Internet connection for UP board over ethernet

##### Procedure
To prepare the UP board device, follow the instructions in sequence:

1. Download [rapyuta.io reference Board Support Package (BSP) image](https://storage.googleapis.com/io-reference-bsp-images/up/ubuntu/2018-08-23-rapyuta-robotics-xenial-ros-up-board-amd64.iso) (amd64 CPU architecture).
{{% notice info %}}  
The custom rapyuta.io image comes with [Ubuntu Xenial](http://releases.ubuntu.com/xenial/)
OS and [ROS kinetic](http://wiki.ros.org/kinetic) software installed on it.
Moreover, the [rapyuta.io tutorials](https://github.com/rapyuta-robotics/io_tutorials)
are also installed on the custom image.
{{% /notice %}}
1. Burn the downloaded image on a USB disk using [Etcher](https://etcher.io/) tool.
2. Insert the USB installer disk into an empty USB port on the device and proceed with the normal Ubuntu installation.
4. Reboot the device once Ubuntu is installed.
5. Connect the ethernet port of the device to a router.

You may use [nmap](https://nmap.org/) to determine the device IP.
Replace *1.2.3.0/24* in the below command with your local IP address.

```bash
nmap -sn 1.2.3.0/24
```
To ensure that everything is OK, SSH into the device. Both the default username
and password for the custom image is ***`rapyuta`***. You are required to change the
password as soon as you sign in.

```bash
ssh rapyuta@1.2.3.0
```

### ARM
#### Preparing Raspberry PI 3
##### Prerequisites

1. Raspberry PI 2 or 3
2. Micro SD card of at least 16 GB and of class 10
3. Internet connection over ethernet

##### Procedure
To prepare the device, follow the instructions step-wise:

1. You may download either [rapyuta.io reference Board Support Package (BSP)
image](https://storage.googleapis.com/io-reference-bsp-images/raspberrypi/ubuntu/2018-07-14-rapyuta-robotics-xenial-ros-raspberry-pi-armhf.img.xz) (arm32v7 CPU architecture) for Raspberry PI 2 and 3,
or [rapyuta.io reference BSP image](https://storage.googleapis.com/io-reference-bsp-images/raspberrypi/ubuntu/2018-07-18-rapyuta-robotics-xenial-ros-raspberry-pi-arm64.img.xz) (arm64v8 CPU architecture) for only Raspberry PI 3.
{{% notice info %}}  
The custom rapyuta.io images come with [Ubuntu Xenial](http://releases.ubuntu.com/xenial/)
OS and [ROS kinetic](http://wiki.ros.org/kinetic) software installed on them.
Moreover, [rapyuta.io tutorials](https://github.com/rapyuta-robotics/io_tutorials)
are also installed on these custom images.
{{% /notice %}}
2. You can easily flash the SD card using [Etcher](https://etcher.io) or other [options](https://www.raspberrypi.org/documentation/installation/installing-images/).
3. Insert the SD card into the Raspberry PI microSD slot and power the device.
4. Connect the ethernet port of the Raspberry PI to a router to access the internet.  
You may use [nmap](https://nmap.org/) to determine the device IP.
Replace *1.2.3.0/24* in the below command with your local IP address.

```bash
nmap -sn 1.2.3.0/24
```
To ensure that everything is working OK, SSH into the Raspberry PI using
the default username and password, both of which are set to ***`rapyuta`***.

You are required to change the password as soon as you sign in.

```bash
ssh rapyuta@1.2.3.0
```

Alternatively, you may use a serial terminal to achieve the same result.
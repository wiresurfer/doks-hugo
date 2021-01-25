---
title: "Setup Device"
description:
type: developer-guide
date: 2019-10-24T11:48:58+05:30
pre: "a. "
weight: 250
---
Copy, paste and run the device's token in the command terminal
of the device. The token sets up the rapyuta.io device client on
the device. Sometimes, you may need root permission to execute
the token on the device.
{{% notice note %}}
You may access the command terminal of the device through either the
serial TTY method or the SSH method.
{{% /notice %}}

If the device is set up successfully, you should see the following output
on the command terminal:

```bash
Initialising the Rapyuta Platform

########(100%)
Successfully installed!
root@ubuntu:/home/ubuntu#
```
The process of installing a device consists of different stages towards
successful completion of onboarding the device on to rapyuta.io.
These stages are described below:

* Checking and installing required python or apt packages.
* Installing ROS messages collector package.
* Installing communication package.
* Installing metrics collector package.
* Installing monitoring package.
* Pulling ROS base image.

Not all devices go through all of the stages in the device initialization
process. For example, while devices with docker-compose option set will
have ROS base image installed on them, those with preinstalled runtime
will have monitoring package installed on them.

{{% notice warning %}}
In case you face issues on-boarding the device. Please refer to the [section on failure codes](/developer-guide/manage-machines/onboarding/setup-device/failure-codes/) to help you troubleshoot.
{{% /notice %}}

When the device is successfully registered, you will see a
<span style="color:green">***green***</span> dot next to the
device's name, which indicates that the device is online.

![Registered Status of Device](/images/getting-started/add-new-device/demo-device.png?classes=border,shadow&width=40pc)


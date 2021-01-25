---
title: "View Device"
description:
type: developer-guide
date: 2019-10-24T11:47:55+05:30
pre: "b. "
weight: 260
---
rapyuta.io assigns a unique identifier, ***UUID***, to the device.

The status of a device is colour coded. The following lists the various statuses
the device may be in:

* Registered (<span style="color:blue">***blue***</span>) indicates that the device is successfully added, but an
active connection is yet to be established.
* Initialising Device (<span style="color:orange">***yellow***</span>) indicates that the device is being initialised.
* Online (<span style="color:green">***green***</span>) indicates that the device is ready to receive commands.
* Offline (<span style="color:grey">***grey***</span>) shows that the device is registered, but is currently offline.
* Rejected (<span style="color:red">***red***</span>) indicates that the device is blocked from communicating with
rapyuta.io
* Delete (<span style="color:red">***red***</span>) indicates that all traces of the device's data is removed from
rapyuta.io
* Failed (<span style="color:red">***red***</span>) indicates that a failure occurred while managing the device.

{{% notice info %}}
You may read the [list of error codes](/developer-guide/manage-machines/onboarding/setup-device/failure-codes/) for more details about specific failure cases.
{{% /notice %}}

On the **Details** page of the device, you may modify its description or
its catkin workspace as shown in the below image.

![Device's Details tab](/images/getting-started/add-new-device/demo-device-details.png?classes=border,shadow&width=60pc)
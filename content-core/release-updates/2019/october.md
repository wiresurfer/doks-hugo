---
title: "October"
description:
type: release-updates
date: 2019-10-30T17:55:46+05:30
weight: 876
---
## October 16
Welcome to the October 16, 2019 release of rapyuta.io platform (v0.31.0).
There are significant updates in this version that we hope you will like.

#### Features

* **Support for custom ROS messages in device metrics**    
  Subscribe to ROS topics of user-defined ROS message types
  in device metrics.
* **Upload device logs to the cloud**    
  rapyuta.io lets you save device logs to the cloud storage,
  thus, enabling you to analyze the logs as per your requirements.
* **Send a bug report from a device's Details tab**    
  Click **Report Device** to raise a bug report from the **Details**
  tab of a device. The device logs are automatically added to the report
  before it is sent to the customer support.
* **rapyuta.io SDK 0.8.0 released**   
  * Added support for logs API

#### Notable Fixes

* Fixed rotation bug in the deployment dependency graph.
* Fixed validation errors related to ROS configuration in package
  manifest.

#### Documentation

* Read about
  [how to upload device logs to the cloud storage](/developer-guide/tooling-automation/logging/device-logs/#batch-upload) in rapyuta.io
* [Download/upgrade](/developer-guide/tooling-automation/python-sdk/#installation) to
  the new rapyuta.io SDK 0.8.0

## October 23
Welcome to the October 23, 2019 release of rapyuta.io platform (v0.32.0).
There are significant updates in this version that we hope you will like.

#### Features

* **Support simulation on the cloud**    
  Simulate robots and their environment using Gazebo simulation running on the cloud in rapyuta.io

#### Update

* **Network endpoint has different domain suffix**    
  Network endpoints of a deployment have a different domain suffix. The old network endpoints will still work.

#### Documentation

* Learn [how to simulate a robot in rapyuta.io](/build-solutions/sample-walkthroughs/turtlebot-simulation/).
* Read more on [simulation using rapyuta.io](/developer-guide/simulation/).

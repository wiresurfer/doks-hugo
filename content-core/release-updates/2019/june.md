---
title: "June"
description:
type: release-updates
date: 2019-10-30T17:55:25+05:30
weight: 884
---
## June 10
Welcome to the June 10, 2019 release of rapyuta.io platform.

This release includes new ROS builder keys for all ROS builds.
The new signing keys are created because the signing key used
for primary ROS repositories was changed after
[investigating a compromising security attack](https://discourse.ros.org/t/security-issue-on-ros-build-farm/9342/7) of
[build.ros.org](http://build.ros.org/)

## June 12
Welcome to the June 12, 2019 release of rapyuta.io platform.
There are a number of significant updates in this version that
we hope you will like.

#### Features
* rapyuta.io lets you to work with git repositories containing
  large files by supporting Git LFS extension.
* Enhanced the reliability of the pipeline for collecting device
  metrics.

#### Documentation
rapyuta.io user documentation is now freely available to all. Even if
you are not registered on [rapyuta.io](https://console.rapyuta.io), you can still access and view its [user documentation](https://userdocs.rapyuta.io).

## June 19
Welcome to the June 19, 2019 release of rapyuta.io platform.
There are a number of significant updates in this version that
we hope you will like.

#### Features
**Configurations** allow you to control the behaviour of a device (robot) or a group of devices based on a large set of parameters.

#### Notable Fixes
Added captcha validation for changing rapyuta.io password, verifing email address and selecting forgot password option.

#### Documentation
Read about the [dynamic configurations](/developer-guide/manage-software-cycle/dynamic-configurations/)
feature, and learn
[how to create and apply a configuration for your devices](/developer-guide/manage-software-cycle/dynamic-configurations/apply-dynamic-configs/).

## June 26
Welcome to the June 26, 2019 release of rapyuta.io platform.
There are a number of significant updates in this version that we hope you
will like.

#### Features
* **Community discussion forum**    
  Users can ask clarifying questions, discuss relevant topics, share
  knowledge and help others by interacting with others in the
  [community discussion forum](https://forum.rapyuta.io/).
* **Multiple credit cards**    
  rapyuta.io supports adding multiple credit cards to an organisation's billing
  account. But, only one credit card is made the primary or default card, which will
  be used for charging fees additional costs.
* **Device system metrics**    
  A set of device system metrics like CPU load average, memory usage, disk usage, disk IO, network IO interface are available for you. These metrics would help you to visualise specific data measurements.

#### Notable Fixes
Fixed a couple of rapyuta.io user interface issues.

#### Documentation
* Read about the
  [community discussion forum](/pricing-support/support/#discussion-forum)
* Read about the newly available [device system metrics](/developer-guide/tooling-automation/metrics/)
* Ensure you are aware of the new
  [multiple credit card support](/pricing-support/pricing/billing-usage/) provided by rapyuta.io
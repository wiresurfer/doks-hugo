---
title: "January"
description:
type: release-updates
date: 2019-10-30T17:52:46+05:30
weight: 898
---
## January 30
Welcome to the January 30, 2019 release of rapyuta.io platform.

This release features newer version of [Python SDK (v0.3.2)](/developer-guide/tooling-automation/python-sdk/#installation).
It adds support for filtering devices by architecture.

## January 23
Welcome to the January 23, 2019 release of rapyuta.io platform. There are a
number of significant updates in this version that we hope you will like.

#### Features
* Support for device error codes    
  When a device goes into the failed state, the corresponding error code is displayed.

#### Notable Fixes
* For packages with **Bindable: True** attribute, fixes the issue of network endpoints not displayed in deployment's details page.
* Fix invalid edges that were being shown in dependency graphs in some cases.
* Add a validation to not let the user add a volume to a cloud component having multiple replicas.
* Fixes deployment failure when an invalid Dockerhub secret exists in either a group or a user scope.
* New UI layout for the *volumes* configuration parameter of a component in deployment's details page.
* Fixes in rapyuta uninstall script.
* Fix for improper device configuration variables being selected when deploying any package with device component.

#### Documentation
* A [list of error codes](/developer-guide/manage-machines/onboarding/setup-device/failure-codes/) when a device is in failed state.
* Revamped [ROS Publisher Subscriber](/build-solutions/sample-walkthroughs/basic-ros-pubsub/preinstalled-runtime/),
[Docker Publisher Subscriber](/build-solutions/sample-walkthroughs/basic-ros-pubsub/docker-runtime/)
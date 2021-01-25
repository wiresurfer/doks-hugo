---
title: "March"
description:
type: release-updates
date: 2019-10-30T17:53:01+05:30
weight: 893
---
## March 18
Welcome to the March 18, 2019 release of rapyuta.io platform. There are a
number of significant updates in this version that we hope you will like.

#### Features

* General availability release with ***Subscription Plans***.
* Introduce projects    
  Change group scope to project scope for better tracking of resources
  within an organization. You must provide unique deployment names and
  device names in a project.
* Introduce memory and compute limits for executables of components
  in a rapyuta.io package.
* Introduce new capacity buckets for volume deployments.
* Introduce Cloud Bridge v5    
  With Cloud Bridge v5, you can provide additional options for multi
  robot communication. A new compression feature over ROS messages is
  available. There are performance improvements.

#### Notable Fixes

* No more user scope for creating rapyuta.io resources such as devices,
  packages, deployments, secrets and package builds.
* The logs of install script are redirected to ***/var/log*** instead
  of ***/tmp***.

#### Documentation

* Describe the newly introduced [Pricing plans](/pricing-support/pricing/)
  for rapyuta.io.
* Explain
  [Multi-robot communication support](/developer-guide/manage-software-cycle/communication-topologies/ros-support/#multi-robot-support) provided by rapyuta.io's Cloud Bridge.
* Update [Python SDK documentation](/developer-guide/tooling-automation/python-sdk/).
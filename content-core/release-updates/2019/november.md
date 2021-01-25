---
title: "November"
description:
type: release-updates
date: 2019-11-06T15:04:39+05:30
weight: 874
---
## November 27
Welcome to the November 27, 2019 release of rapyuta.io SDK.

#### Features

**rapyuta.io SDK 0.9.1 released**    

* Improved ***poll_deployment_till_ready*** method for ***Deployment*** and ***VolumeInstance*** classes.
* Fixed minor formatting issues in [SDK reference](https://sdkdocs.apps.rapyuta.io/).

#### Documentation

* UI enhancements like hide-sidebar button, mobile optimizations, support for search engine indexing, and other minor fixes.
* Read the new walkthrough on [separating a navigation application and its simulation](/build-solutions/sample-walkthroughs/separate-navigation-simulation/) on rapyuta.io
* Read about the [support for custom ROS messages](/developer-guide/tooling-automation/metrics/ros-support/).

## November 13
Welcome to the November 13, 2019 release of rapyuta.io platform (v0.34.0).
There are significant updates in this version that we hope you will like.

#### Features

**Import packages to rapyuta.io**    
Import packages to rapyuta.io by uploading, or pasting JSON package
manifests in the editor, or from external JSON URLs.
  
When an external JSON URL redirects to rapyuta.io, the redirect
URL is constructed with ***uo*** and ***link*** parameters:

***https://console.rapyuta.io/catalog?uo=1&link=<JSON URL>***
  
Consequently, the editor will be auto-filled with the
data from the JSON URL. 
  
This feature is useful when you have a JSON package manifest ready,
and would not want to manually enter data values for various fields
while creating packages.

#### Notable Fixes

* Added validation check for the length of file name while uploading
  logs.
* Fixed internal bugs for rapyuta.io's device agent while onboarding
  a device on to rapyuta.io platform.
* Fixed minor user interface bugs.

## November 06
Welcome to the November 06, 2019 release of rapyuta.io platform (v0.33.0).
There are significant updates in this version that we hope you will like.

#### Features

* **Show/hide deployments in a dependency graph**    
  Toggle between show/hide of stopped deployments in a dependency graph.
* **Deploy local communication broker with preferred network interface**    
  Ability to specify a preferred network interface for local communication broker when added as dependent deployment.

* **rapyuta.io SDK 0.9.0 released**
  * Added ability to create package from manifest file and manifest dictionary.
  * Added ability to connect preferred network interface of the local communication broker when local broker added as dependent deployment
  * Added *packageVersion* field to *Package* class.
  * Added ability to propagate error from REST client.
  * Rename mission to selection API.

#### Notable Fixes

* Fixed bug while reporting errors of deployments and devices.

#### Documentation

* A complete overhaul of user documentation.
* Read about [specifying network interfaces while deploying the local
  communication broker](/developer-guide/manage-software-cycle/communication-topologies/local-communication-broker/).

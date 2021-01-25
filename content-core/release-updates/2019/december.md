---
title: "December"
description:
type: release-updates
date: 2019-12-04T15:09:13+05:30
weight: 872
---
## December 04
Welcome to the December 04, 2019 release of rapyuta.io platform (v0.35.0).
There are significant updates in this version that we hope you will like.

#### Features

* **Visualization of device onboarding stages**    
  The process of onboarding a device is made up of different stages. A progress bar indicates the on-going and completed stages of the process.
* **Support timeout for ROS services**    
  Set a timeout when adding a ROS service to a rapyuta.io package. It is the number of seconds to wait for a ROS service response before timing out.

#### Notable Fixes

* Fixed disappearing blinking dots indicating build status of a package.
* Improved the reliability of WebSSH. There will be lesser frequent disconnections.
* Fixed terminal resize issue of WebSSH.

#### Documentation

* Read about the [different stages of the device initialization process](/developer-guide/manage-machines/onboarding/setup-device/).
* Added a new device failed error code: [DEV_E108](/developer-guide/manage-machines/onboarding/setup-device/failure-codes)

## December 11
Welcome to the December 11, 2019 release of rapyuta.io platform (v0.36.0).
There are minor updates in this version that we hope you will like.

#### Notable Fixes

* Fixed the issue of redirecting to rapyuta.io dashboard on sign in every time the user signs out of rapyuta.io

#### Documentation
Open the user documentation from the [home page](https://console.rapyuta.io/) of rapyuta.io
![User Documentation](/images/updates/user-docs-link.png?classes=shadow,border&width=50pc)

## December 18
Welcome to the December 18, 2019 release of rapyuta.io Python SDK 0.10.0

#### Features

**rapyuta.io Python SDK 0.10.0 released**

* **Added methods for interacting with configuration parameters**    
  Use ***upload_configurations()*** and ***download_configurations()*** methods to upload and download configuration parameters using the SDK.
* **Added method for applying configuration parameters to devices**    
  Use ***apply_parameters()*** method for applying configuration parameters to devices using the SDK.

## December 24
Welcome to the December 24, 2019 release of rapyuta.io platform.

#### Feature

**Access user-defined endpoints as environment variables**    
Access user-defined endpoints in a deployment of a rapyuta.io
package using environment variables via the deployment's
shell access option.
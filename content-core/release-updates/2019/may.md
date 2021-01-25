---
title: "May"
description:
type: release-updates
date: 2019-10-30T17:53:44+05:30
weight: 886
---
## May 27
Welcome to the May 27, 2019 release of rapyuta.io platform.
There are a number of significant updates in this version
that we hope you will like.

#### Features
* You can now clone a package to a different project.
* While creating a package, you can provide git repository URLs
  of the form ***git@github.com:user/path***
 
#### Notable Fixes
* Fixed bugs in streaming and live logs.
* Added validation for git repository URLs in package creation process.
* Added validation to check for duplicate cloud network endpoint names in package creation.
* Fixed bug in the process of creating multiple cloud network endpoints.
* Fixed the bug in deployment details page not showing multiple dependent deployments
* Fixed few bugs in deployment creation.
* Improved user analytics.


## May 15
Welcome to the May 15, 2019 release of rapyuta.io platform.
A couple of bugs are fixed in this release for under-the-hood
improvements.

## May 08
Welcome to the May 08, 2019 release of rapyuta.io platform.
There are a number of significant updates in this version
that we hope you will like.

#### Notable Fixes
* Added a new deployment error code, ***DEP_E4xx***    
  **DEP_E4xx** is introduced for ***failed to start*** deployments. Internal errors like network/HTTP connection timeout or database operation failure are not displayed on the deployment page. Instead ***Internal rapyuta.io error*** is shown to the user.
* Fixed the issue to not allow empty keys for device configuration variables.
* Enforced additional constraints on names of devices.
* No duplicate device configuration variables are allowed.
* Released a new version of ***rapyuta-agent*** with minor changes for devices. If your device is already onboarded, it is recommended to update the device by re-boarding it using the lastest ***rapyuta-agent***.

#### Documentation
* Added [API Documentation](https://gadocs.apps.rapyuta.io/).
* Add [Basic Web Application](/build-solutions/sample-walkthroughs/basic-web-app/) tutorial to illustrate dockerfile build strategy.
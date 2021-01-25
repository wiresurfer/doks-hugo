---
title: "December"
description: 
type: release-updates
date: 2020-12-02T15:00:22+05:30
weight: 820
---


## December 02
Welcome to the December 02, 2020 release of the rapyuta.io platform. There
are significant updates in this release that we hope you will like.

#### Features
* **Historical logs new UI**

    A new UI is available for the historical logs and paginated download option for the historical log lines. For more information, [click here](/developer-guide/tooling-automation/logging/deployment-logs).

* **Docker Multistage Builds**

    Multistage builds are useful to anyone who has struggled to optimize Dockerfiles while keeping them easy to read and maintain. For more information, [click here](/developer-guide/create-software-packages/builds/#docker-mulitstage-build-support).

#### Improvements
	
- Better internal monitoring of cloud routed networks


#### Bugfixes

- Fixes and improvements around Historical Logs Download feature
- Fixes on APIs. 

## December 09

Welcome to the December 09, 2020 release of the rapyuta.io platform. There
are significant updates in this release that we hope you will like.

#### Features
* **Pagination on Manage tab**

    The rapyuta.io platform displays the uploaded device log files in a paginated view and allows you to customize the view. For more information, [click here](/developer-guide/tooling-automation/logging/device-logs/#viewing-uploaded-log-files).


#### Improvements
	
- Improved **Apply Parameters** performance allowing the user to download binary files faster.

#### Notable Fixes

- When you try to cancel the upload of a [device upload-log](/developer-guide/tooling-automation/logging/device-logs/#batch-upload) file on the *rapyuta.io* platform, the cancellation fails with *FAILED CANCELLING LOG UPLOAD* error message. Fixed the bug that caused this error.

#### SDK

**rapyuta.io Python SDK [0.18.0](/developer-guide/tooling-automation/python-sdk/#installation) released**

- Added support for build docker push secrets and repositories.
- Added support for creation of shared URLs for device logs.
- Added support for project and secret management. Also added support for *get_auth_token()* and *set_project()* methods.
- Added retries in case of GET requests that fail with internal server error.


## December 30

Welcome to the December 30, 2020 release of the rapyuta.io platform. There
are significant updates in this release that we hope you will like.

#### Features

* **Cloning a Build**

The rapyuta.io platform allows you to clone a build within the same or different project and reduces the time and effort required to create a build from scratch. For more information, [click here](/developer-guide/create-software-packages/builds/build-cloning).

#### Improvements
	
- Added new device error code, DEV_E110. For more information regarding the error description and troubleshooting, [click here](/developer-guide/manage-machines/onboarding/setup-device/failure-codes/).


#### SDK

**rapyuta.io Python SDK [0.18.1](/developer-guide/tooling-automation/python-sdk/#installation) released**

- Fixed retry_limit parameter in ProvisionClient.
- Fixed the attribute error that was occuring while provisioning the same package with different deployment name and same configuration parameter.


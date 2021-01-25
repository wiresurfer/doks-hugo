---
title: "February"
description:
type: release-updates
date: 2020-02-05T15:40:05+05:30
weight: "837"
---
## February 05
Welcome to the February 05, 2020 release of the rapyuta.io platform. There are significant updates in this release that we hope you will like.

#### Features

* **Override restart policy option**    
Modify or override the initial setting of restart policy for components with device runtime while deploying a package.
* **Rapyuta IO Communication Broker package's restart policy**    
The restart policy for **Rapyuta IO Communication Broker** package is ***Always***.
* **Static routes**    
To get a deterministic URL/route for your application while exposing the network endpoint externally, you must bind it to a static route. When you add a static route, an externally exposed endpoint is essentially guaranteed to be available at the URL of that particular static route. It makes externally exposed endpoints (and hence the deployments exposing them) resilient to failure or re-deployment, facilitates maintenance and upgrades to the backend/deployment while retaining the same unique globally available URL.
* **rapyuta.io Python SDK 0.11.0 released**    
Python SDK [0.11.0](/developer-guide/tooling-automation/python-sdk/#installation) defines a method that adds restart policy for a device component.
To use newer SDK, install it by using:

```bash
pip install https://storage.googleapis.com/rio-sdk-python/rapyuta_io-0.11.0-py2-none-any.whl
```

#### Improvements

* Improved the logging infrastructure.
* The default filter value for device logs is set to all.

#### Notable Fixes

* Fixed install and uninstall script giving false ***deployments running*** error.

#### Documentation

* Read about overriding [restart policies](/developer-guide/manage-software-cycle/deployments/#restart-policy).
* Read about [static routes](/developer-guide/create-software-packages/package-internals/#exposing-endpoints-with-static-url) feature and its effect on [subscription plans](/pricing-support/pricing/find-plans/).

## February 12

Welcome to the February 12, 2020 release of rapyuta.io platform. This release includes several internal improvements, optimizations and bug fixes.

#### Improvements
The rapyuta-agent services now have a delay between restarts in case of failure.

## February 19
Welcome to the February 19, 2020 release of rapyuta.io platform. There are significant updates in this release that we hope you will like.

#### Features

* **New system metric: Wireless**    
  Subscribe to the new system metric, ***Wireless***, for devices. It provides information on the WiFi signal strength.
* **Error logs for device metrics**    
  A new panel for viewing error logs when subscribing/unsubscribing to metrics of a device is added.
  ![Metrics error logs](/images/chapters/developer-guide/tooling-automation/metrics/metrics-error-logs.png?classes=border,shadow&width=50pc)
* **rapyuta.io Python SDK 0.12.0 released**    
  * Defines a new system metric for subscription: Wireless
  * Fixed typographical errors in the SDK documentation

```bash
pip install https://storage.googleapis.com/rio-sdk-python/rapyuta_io-0.12.0-py2-none-any.whl
```

#### Improvements

* Improved the UI for editing the name of a device.
* Improved the UI for updating or editing device configuration variables on the device details page.

#### Notable Fixes

* Fixed sort-by-action on the deployment list page.
* Fixed use of ***ESC*** key to toggle fullscreen when remotely accessing cloud deployments or devices, which interfered with terminal applications like *vim*. Now, use ***CTRL+ESC*** to toggle fullscreen. 
* Fixed the issue in the help and support form. The form will be automatically filled with the registered email address.
* Fixed the shorter timeout value when remotely accessing cloud deployments and devices on rapyuta.io; it is increased to 15 minutes.

#### Documentation
Read more about the new panel that displays error logs when subscribing/unsubscribing metrics of a device [here](/developer-guide/tooling-automation/metrics/ros-support/#type-introspection-and-changing-data-types).

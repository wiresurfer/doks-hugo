---
title: "Core Concepts"
description: "Learn about the resources which can be provisioned on the rapyuta.io platform."
lead: "Learn about the resources which can be provisioned on the rapyuta.io platform."
date: 2020-10-13T15:21:01+02:00
lastmod: 2020-10-13T15:21:01+02:00
draft: false
images: []
menu: 
  docs:
    parent: "overview"
weight: 105
toc: true
---

## Package
A package is a fundamental rapyuta.io *resource* that represents a
declaration of your application. It is the smallest unit of deployment
in rapyuta.io, and it can be deployed either on a device or the cloud
or both.

To make this possible a package must encapsulate any information about how
it should be built, its compatibility and runtime requirements,
network endpoints and ROS interfaces it exposes, and any configuration
information it may require.

{{% notice info %}}
Learn about the internals of a package [here](/developer-guide/create-software-packages/package-internals/).
{{% /notice %}}

## Builds
Builds on rapyuta.io are a fundamental resource which convert your source code
residing in your VCS into a container image.

Builds can be referenced when creating packages and enables an 
end-to-end "Code to Deployment" pipeline for your Robotics solution.

{{% notice info %}}
Learn about Builds [here](/developer-guide/create-software-packages/builds/).
{{% /notice %}}


## Catalog
The catalog serves as the portal in rapyuta.io to streamline the software
lifecycle management allowing you to spawn deployments, create new
packages, and maintain package versions and updates. Additionally, it features
a built-in collection of storage, communication, and curated platform
packages that you can put together to build your solution.

It is also responsible for implementing design patterns that allow a user
to leverage powerful build-time or run-time semantics to combine multiple
packages and compose new behavior.

{{% notice info %}}
Learn more about the package catalog [here](/developer-guide/create-software-packages/package-catalog/).
{{% /notice %}}

## Deployment
A deployment is a *rapyuta.io resource* that represents a unique
instantiation of a rapyuta.io package. It holds information
about the package deployed, the configuration used, and interfaces
exposed. It possesses a unique identifier and provides a mechanism
to introspect its phase and state that are needed to ascertain
the state of a system.

Tooling such as logs, debug terminals and other automation leverage
this uniquely identifiable resource to allow the operator to manage,
debug and observe a particular running instance of their application.

Deployments support composition patterns to allow the user to
combine multiple different applications to help realize a potentially more 
complex robotics solution.

{{% notice info %}}
Learn more about deployments [here](/developer-guide/manage-software-cycle/deployments/).
{{% /notice %}}

## Device
A device is a rapyuta.io resource representing any physical device
that typically lives at a client location and is registered on
rapyuta.io.
The resource encapsulates information about the device, its architecture,
runtime, user-provided metadata. Once a particular piece of hardware
is successfully on-boarded to rapyuta.io this entity is responsible
for providing the necessary mechanics and communication channels
to manage and interact with the device. The platform leverages
these mechanics to provide features that can be used to communicate
to the device, configure it, monitor its health and deploy packages
to the device.

{{% notice info %}}
Learn more about devices [here](/developer-guide/manage-machines/).
{{% /notice %}}
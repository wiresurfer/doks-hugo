---
title: "Device Runtime"
description:
type: developer-guide
date: 2019-10-24T11:40:10+05:30
pre: "1. "
weight: 230
---
Different developers build and manage software for devices
differently. rapyuta.io supports two primary ways for
application developers to manage software on a device. This
enables developers the freedom to use the current software
with minimal changes, if any, as well as choose to leverage
an entirely new runtime for a new generation of software.
rapyuta.io defines these ways as device runtimes.

### Preinstalled
The preinstalled runtime is designed to support existing
software that is already *preinstalled* on the device.
The existing software may be delivered by the traditional
system images, package manager or by other means,
additionally, this software is often managed by the custom
solutions that software initialization.

rapyuta.io makes it possible for the developer to leverage
the numerous innovative features of the platform composition,
automation, management and tooling functionality of the platform
without having to rework an existing workflow.

### Containerized: Docker Runtime
Containers offer a logical packaging mechanism in which
applications can be abstracted from the environment in
which they run. This decoupling allows container-
based applications to be deployed easily and consistently,
regardless of whether the target environment is the public
cloud, or a connected device. Containerization provides
a clean separation of concerns, as developers focus on
their application logic and dependencies. Hence, it is
possible to offer critical features for distributed
connected robotics like transactional software updates,
rollback releases and delivers content signed software
to ensure consistent operation across multiple robots
in multiple locations.

rapyuta.io has first-class support for containers on the
device and the cloud. It strives to maintain identical
workflows, tools, and interfaces while providing tools
like the build engine that enables developers to leverage
these new workflow patterns like gitops for robots.

To set docker-compose as the default runtime on a device
select **Use docker compose as default runtime** checkbox
while adding the device.

![Dockercompose runtime for device](/images/core-concepts/device-management/docker-runtime-for-device.png?classes=border,shadow&width=40pc)

When a package is deployed on a device, each executable
in the package becomes a separate docker container. If your
application is a ROS package, a singleton ROS Master starts in
an individual docker container different from that of the executables.

All docker containers that are deployed are configured to
use the host network driver. With the host network driver, the
container’s network stack is not isolated from the host. For example,
if you run a docker container that uses port 80, the container’s
application will be available on port 80 on the host’s IP address.

{{% notice note %}}  
ROS_MASTER_URI on a device using docker runtime is  http://<*hostname*>:1234   and not <*localhost/ROSNAME*>:11311
{{% /notice %}}
---
title: "Design Patterns"
description:
type: developer-guide
date: 2019-10-25T12:51:38+05:30
pre: "b. "
weight: 420
---
The Catalog supports package composition - combining two or more packages to create a new package. Composing packages requires binding the packages together. If a package depends on another package or the deployment of another package, its bindings link to its dependencies.

## Build Time Dependencies: Include Packages
A package binding is a set of configuration parameters and network endpoints that a package exposes to its dependencies. rapyuta.io implicitly injects the
exposed parameters and network endpoints into packages, which depend on another
package or its deployments, as environment variables.

For example, a _database_ package exposes a network endpoint - *HOST_AND_PORT*
(endpoint with both host IP address and port number), and the configuration
parameters - _USER_, _PASSWORD_ and *DATABASE_NAME*. Any package that depends on the
_database_ package may use the exposed parameters and network endpoint.
A *robot_management* package binds to the _database_ package if the former package relies on the deployment of the latter package. Then, the package bindings of the dependent deployment (in this case, _database_) are automatically injected into the running instance of the *robot_management*.
It implies that all of the executables of the *robot_management* may consume the *HOST_AND_PORT*, _USER_, _PASSWORD_, and *DATABASE_NAME* as environment variables.

While adding a package, if you need to compose one or more packages in a specific behavior that requires **tight coupling** of those packages including their life cycles and instantiation, then you should add those packages from the service catalog.

For example, a robot such as an Automated Guided Vehicle (AGV) or a drone requires a real time video streaming gateway. It will be useful to abstract the video streaming logic and the underlying complex infrastructure into a *simple_video_gateway* package so as to make it reusable for other robots.

Suppose the robot is an AGV. Then, there exists an *agv_pkg* package that controls the motion of the AGV in a 2D plane and includes the *simple_video_gateway* without
having to deal with the additional complexities of real-time video processing.
Similarly, if the robot is a drone, the *drone_pkg* focuses on the essential
functions of retrieving precise direction, altitude, pose control, etc. and
includes *simple_video_gateway* without having to worry about the additional
complexities of real-time video processing.

The *simple_video_gateway* is implicitly provisioned every time *drone_pkg* or
*agv_pkg* is deployed.

When a package, say _PkgA_ is included in another package say _PkgB_, the configuration
parameters and network endpoints that are exposed by _PkgA_ are automatically
injected into the components of _PkgB_ as environment variables.

The components of _PkgB_ might have configuration parameters that are stored as
environment variables in the deployment of _PkgB_. In this case, there is a
possibility of collisions that rapyuta.io does not handle at this moment.

A *singleton package* is one that is instantiated only on its first deployment request.

## Run Time Dependencies: Dependent Deployments
Dependent deployment allows you to control the life cycle of a set of components independently, and bind them during package instantiation. You may use this design pattern when you need to compose one or more packages in a specific way that requires **loose coupling** of those packages with different life cycles such as coupling between multiple short lived agents and a long running shared service.

For example, a component, say *cloud_mapping_service* maintains a global map of
your warehouse. It is a long running service, which is used by multiple robots
for retrieving information about their environment and for updating the map with positions of obstacles for their neighboring robots. When a robot comes online, its *agv_bot* component binds to the *cloud_mapping_service*, and then it proceeds to perform its tasks. The deployment ID and name are used to add a given deployment as a dependency to *agv_bot*. At this point, all ROS topics, ROS services, and ROS actions of *cloud_mapping_service* are locally available in every deployment of *agv_bot*. However, the inbound ROS interfaces of the *cloud_mapping_service* require you to provide a white list of ROS topics, ROS actions, and ROS services that are allowed from the deployments of *agv_bot*.
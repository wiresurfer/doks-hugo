---
title: "Binding"
description:
type: developer-guide
date: 2019-10-25T12:51:13+05:30
pre: "a. "
weight: 415
---
Catalog supports package composition - combining two or more packages
to create a new package. Composing packages requires binding the
packages together into one package. If a package depends on another
package or deployment of another package, the package bindings link
the package to its dependencies.

A package binding is a set of configuration parameters and network endpoints
that a package exposes to its dependencies. rapyuta.io implicitly injects the
exposed parameters and network endpoints into packages, which depend on another
package or its deployments, as environment variables.

For example, a _database_ package exposes a network endpoint - *HOST_AND_PORT*
(endpoint with both host IP address and port number), and the configuration
parameters - _USER_, _PASSWORD_ and *DATABASE_NAME*. Any package that depends on the
_database_ package may use the exposed parameters and network endpoint.
A *robot_management* package binds to the _database_ package if the former package relies on deployment of the latter package. Then, the package bindings of the dependent deployment (in this case, _database_) are automatically injected
into the running instance of the *robot_management*. It implies that all of
the executables of the *robot_management* may consume the *HOST_AND_PORT*, _USER_,
_PASSWORD_ and *DATABASE_NAME* as environment variables.

## Bindable Attribute
A boolean attribute that is set while adding a package. When set to true for a
package with two or more ROS components, the components successfully communicate through [ROS interfaces](/developer-guide/create-software-packages/ros-support/#ros-native-communication-interfaces).

It also determines whether a deployment of a package can be used as a dependent deployment of other deployments. If set false, the deployment of the package cannot be used as a dependent deployment of another deployment.

It also determines whether a package can be added as an include package. If set
false, the package cannot be used as an include package.

## Communication Linking
To facilitate communication between deployments that are bound, rapyuta.io implements various communication topologies. To learn more of the implication please refer the section on [**communication topologies**](/developer-guide/manage-software-cycle/communication-topologies/).
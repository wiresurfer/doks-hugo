---
title: "Software Architecture"
description:
type: dev-tutorials
date: 2018-11-23T10:46:51+05:30
pre: "1. "
weight: 101
---
rapyuta.io recommends a modularized software architecture design. This
implies responsibilities are divided across various packages because a
modularised software is easy to maintain, share, and scale.

Turtlesim is composed of four packages. They are:

* **User Interface**     
  An interactive, web-based user interface for visualizing the state of the
  Turtles and for passing control commands to the Turtles.
* **Command Center**     
  A central node for registering Turtles, as well as a router for passing
  commands and telemetry messages between the User Interface and the Turtles.
* **Turtle**       
  A package that emulates the behavior of an autonomous ground vehicle with
  [non-holonomic](https://en.wikipedia.org/wiki/Nonholonomic_system) constraints and two distance sensors through the Simulator.
  There can be multiple instances of this component, each representing a
  single robot entity.
* **Simulator**        
  A package that simulates the physical dynamics of 2D robots and their sensors.

The below diagram illustrates how these packages are interconnected.
The arrows represent the direction of information flow between the package.

![Interconnected packages](/images/tutorials/turtlesim/interconnected-packages.png?classes=border,shadow)

In ROS terminology, each package (apart from the User Interface) is a [node](https://wiki.ros.org/rosnode).
The following diagram illustrates the information that is passed between
the nodes.

![Packages as Nodes](/images/tutorials/turtlesim/packages-as-nodes.png?classes=border,shadow)

Each of these nodes (including the User Interface) will be deployed as a
package on rapyuta.io. These packages will pass the same information to one
another, except that they will do so in the cloud.

## Interfaces
Defining interfaces for a package is one of the critical steps during package
creation. A package's interface determines how it may interact with other packages.

You can define component-level interfaces such as endpoints, ROS topics
(publisher), ROS services and/or ROS actions. Furthermore, you can define
package-level interfaces such as ROS topics (subscriber), ROS services
and/or ROS actions.

### Package-level inbound interface vs Component-level interface
There may be multiple instances (or deployments) of Turtle. Each Turtle
instance publishes the /sim/cmd_vel ROS topic. The Simulator subscribes to
the /sim/cmd_vel topic. You'll define an inbound ROS topic interface for
/sim/cmd_vel to help Simulator recognize the different Turtle instances.

In general, if you have a Package-A that expects data from Package-B, and
there can be multiple deployments of Package-B, you must list the expected
data interfaces as inbound interfaces at the package-level.

## Dependent deployment
Package A depends on Package B, that is Package B is a dependent deployment of
Package A. The arrow in the diagram represents the dependency relationship.
Package B injects configuration parameters and/or network endpoints into
Package A as environment variables.

![Modal dependent deployment](/images/tutorials/turtlesim/dependent-deployment.png?classes=border,shadow)

Dependent deployment is a design pattern when you want to loosely couple
packages having different instantiated life cycles.

A deployment of User Interface relies on that of Command Center. Hence,
Command Center is a dependent deployment of the User Interface. Similarly, a
deployment of Command Center depends on that of Simulator, and therefore,
Simulator is a dependent deployment of Command Center. However, Simulator is
independent of any other deployment.

The table summarises dependency relationships between all of the packages
in Turtlesim tutorial.

| Package | Dependent Deployment(s) |
| ------- | ----------------------- |
| Simulator | None |
| Command Center | Simulator |
| User Interface | Command Center |
| Turtle | Command Center, Simulator |

The below diagram illustrates the dependencies among packages in Turtlesim tutorial.

![Dependent deployments block diagram](/images/tutorials/turtlesim/turtlesim-ddeploy-blk-diagram.png?classes=border,shadow)

You may find it instructive to execute the Turtlesim tutorial locally on
your machine. A reference implementation together with execution instructions
is found in the [GitHub repository](https://github.com/rapyuta-robotics/io_tutorials/tree/master/io_turtlesim) and [Docker registry](https://hub.docker.com/r/rrdockerhub/io_turtlesim_ui/).

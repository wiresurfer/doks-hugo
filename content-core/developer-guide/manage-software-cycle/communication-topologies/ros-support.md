---
title: "ROS Communication"
description:
type: developer-guide
date: 2019-10-25T12:54:26+05:30
pre: "b. "
weight: 435
---
The rapyuta.io platform relies on a sub-component called the *cloud bridge*
for implicitly establishing a communication channel between two or more
ROS environments. It is an application-level bridge that offers many
compelling features to ROS developers including
[augmented ROS over the public internet](/developer-guide/create-software-packages/ros-support/#augmented-ros) and dedicated features for dynamic
multi-robot ROS communication.

The below list of topics are explained:

- [Multi-Robot ROS Communication](#multi-robot-ros-communication)
  - [Illustrating a Multi-Robot Scenario](#illustrating-a-multi-robot-scenario)
  - [Dynamic Multi-Robot Communication Semantics](#dynamic-multi-robot-communication-semantics)
    - [ROS Environment Aliases: runtime identity assignment](#ros-environment-aliases-runtime-identity-assignment)
    - [Discovering peers in an ensemble](#discovering-peers-in-an-ensemble)
    - [Scoping: auto prefix or namespace by self identity](#scoping-auto-prefix-or-namespace-by-self-identity)
    - [Targeting: auto prefix or namespace unwrapping for peers](#targeting-auto-prefix-or-namespace-unwrapping-for-peers)
    - [Targeting and Inbound ROS Interfaces](#targeting-and-inbound-ros-interfaces)
- [Automatic Linking of ROS Interfaces](#automatic-linking-of-ros-interfaces)


## Multi-Robot ROS Communication
The rapyuta.io platform offers an elegant solution for multiple robots
communication as a primary feature. 
In the rapyuta paradigm, the component of each package is treated as
an isolated ROS environment.
While declaring the package you are only required to provide the
topics/services/actions required to be exposed by that particular
component. The platform is then responsible for connecting, managing
and securing the ROS environments together. We introduce a set of
new features aimed at making it a lot easier to use multiple robots.

### Illustrating a Multi-Robot Scenario
To illustrate a scenario involving multiple robots we turn to an example
involving the world's favorite sport - soccer. 
Similar topologies are often relevant in real-world applications of robots
such as warehouses where a ***coordinator*** controls multiple AMR/AGVs 

Imagine a game of robot soccer where players are robots, and their coach
is a controller unit. 

The players follow a simple convention

* A player moves to a position when it receives a message on the */move* topic
* A player publishes its pose and location through */odom*.

Now if the coach (controller) needs to use this information from all
players(robots) in the field.
To deal with multiple players(robots) it is necessary to create a
convention that allows him to specifically 
access information about one specific player(robot) and issue commands
to one specific player(robot). 

In the ROS community, the common approach used in multi-robot communication
scenarios is to __prefix or namespace__ the interfaces (topics/services/actions)
__by a unique identity__ typically the name of the robot. 

Following this convention, if the coach(controller)

*  wants to move _robot A_ in a specific direction, it must explicitly publish */robotA/move* that is subscribed by _robot A_ (or */robotB/move*
and */robotC/move* to robot B and robot C respectively)
* wants to seek odom from _robot A_, it must explicitly subscribe to */robotA/odom* that is published by _robot A_  (or */robotB/odom*
and */robotC/odom* from robot B and robot C respectively)

This isvachieved using carefully crafted launchfiles using
remaps (e.g. /move to /robot_A/move), conditionals (e.g. unless ns!="" or
if robot_name=="robot_A"), arguments(e.g. robot_name:=robot_A) and
namespaces(e.g. &lt;node ns=robot_A&gt;). This mandates the delicate
arrangement of files is frozen while building the software and
consistently distributed to all involved agents. As the needs/software change
and the number of variables and robots increase, this approach becomes increasingly
error-prone.


![Robot soccer block diagram](/images/multi-robot-communication/robotSoccer-blk-diagram.png?classes=border,shadow&width=50pc)

### Dynamic Multi-Robot Communication Semantics
Avoid complex hardcoded logic in launchfiles that lives with the source code
or binary and automatically add/remove prefixes to ROS interfaces(topics/services/actions).
Additionally, it is more flexible/dynamic to assign and use deploy-time identities
than hard-coded robot names. 

The process of assigning an identity to a robot and the mechanisms to
consume/discover identities of all alive robots is described in the [ROS environment aliases](#ros-environment-aliases-runtime-identity-assignment) topic.

The mechanisms and features offered by the platform to deal with automatic prefix addition and removal is described in the
[scoping](/developer-guide/manage-software-cycle/communication-topologies/ros-support/#scoping-auto-prefix-namespace-by-self-identity) and
[targeting](/developer-guide/manage-software-cycle/communication-topologies/ros-support/#targeting-auto-prefix-namespace-unwrapping-for-peers) topics.

#### ROS Environment Aliases: runtime identity assignment 
When __deploying a component__ to a robot in a multi-robot scenario,
the platform expects the user to input a unique alias per component
, thereby, assigning a stable identity to that deployment for the
period of its lifetime.

This __alias__ defined at deploy-time __uniquely identifies__ the robot
in the __scope of an ensemble__ of connected peers.

All __components in the same package__ (each potentially deployed in a
physically different device/location) or __linked dependent deployments__
are considered a __part of an ensemble__.

The platform detects duplicates aliases explicitly for the case of
components and deployment and its immediate parent. 

{{% notice info %}}
This alias is available to Deployment [Executables](/developer-guide/create-software-packages/package-internals/#executables)
(both cloud and device) through **RIO_ROS_ENV_ALIAS** environment variable.
{{% /notice %}}

{{% notice note %}}
In the case of siblings (two deployments depending on the same parent), the component with
a duplicate alias is considered a conflict and may enter an error
state. The user must be careful and ensure the uniqueness of identities
present in this case.
{{% /notice %}}

#### Discovering peers in an ensemble
rapyuta.io automatically exposes a latched ROS topic
__/rapyuta_io_peers__ of type *std_msgs/String*, which contains the
list of all connected peers aliases, the first
entity is always the alias of the local bridge.

For example, **My_alias,peer_1,peer_2**

The end-user can potentially use this in their ROS applications for
discovering connected robots.

{{% notice note %}}
There may be more than one bridge locally depending on how you
deployed the packages and components so you may receive multiple
messages. In each case, the first element is always the alias of the
bridge that published this message.
{{% /notice %}}

#### Scoping: auto prefix or namespace by self identity
In this configuration, a user may declare a topic/service/action
as ***scoped*** by selecting the **Scoped** option. 

This indicates that when a component is deployed its local ROS interfaces
(topic/service/action) get __automatically prefixed/namespaced bits own dynamic identity__
(its own ROS environment alias) __as seen by all other robots/peers__ in
the ensemble in their respective ROS graph.

For example, suppose robotA publishes */odom* topic in its local
ROS environment. While routing */odom* to either of the robot
peers (for instance a controller) the topic is prefixed with that
specific robot’s name, in this case, */robotA/odom*.

![Scoped topic](/images/multi-robot-communication/scoped-topic.png?classes=border,shadow&width=50pc)

You can express ***scoped*** as:
A scoped topic is a mapping from a /topic to /robot-peer-name/topic.
{{% notice info %}}
**/odom -------> /robotP/odom**
{{% /notice %}}

![Scoped topic as shown](/images/multi-robot-communication/scoped-as-shown.png?classes=border,shadow&width=50pc)

{{% notice note %}}
If in the ROSmsg logs you experience the error: ***incoming connection failed: unable to receive data from sender, check sender's logs for details***, please ignore it. The error message is generated by ROS internally as a side effect of the sniffing done by the cloud bridge so as to determine metadata related to ROS message type for the service. It has no other effects on your code and/or the code's functionality, and you can safely ignore it.
{{% /notice %}}

#### Targeting: auto prefix or namespace unwrapping for peers
In this configuration, a user may declare a topic/service/action as ***targeted***
by selecting the **Targeted** option. 

This indicates that when a component is deployed its local
ROS interfaces (topic/service/action)
__containing a prefix/namespace corresponding to another individual peer's dynamic identitiy__
(peers ROS environment alias)
gets __routed to the corresponding peer__ and
__automatically unwraps the prefix/namespace__ in its ROS graph.

For example, suppose robots in a particular scenario subscribe to the
topic */cmd_vel* to move and a central controller needs to ask a
specific robot, say robotA, to move, then it needs to be able to target
only robotA and send messages to its */cmd_vel* subscription.

The controller in the above scenario publishes */robotA/cmd_vel* topic.
While routing */robotA/cmd_vel* the bridge strips the prefix ***robotA***
and publish the messages on the topic ***/cmd_vel*** in robotA’s local ROS
environment.

![Targeted topic](/images/multi-robot-communication/targeted-topic.png?classes=border,shadow&width=50pc)

You can express ***targeted*** as:
A targeted topic is a mapping from /robot-alias/topic to /topic.
{{% notice info %}}
**/robotP/cmd_vel -----------> /cmd_vel**
{{% /notice %}}

![Targeted topic as shown](/images/multi-robot-communication/target-as-shown.png?classes=border,shadow&width=50pc)

#### Targeting and Inbound ROS Interfaces
When a package allows for [inbound ROS interfaces](/developer-guide/create-software-packages/ros-support/#inbound-interfaces), you must provide hints to leverage the automatic
targeting feature. The platform introspects the package to determine if it must enforce the unique identity constraints required for multi-robot communication.

As the platform follows a provider only semantic, determining this is
straightforward for ***scoped*** as it is based on the identity of the deployment
itself. It gets complicated for targeting as this depends on the identity of
other peers. When a package is the first to be deployed (or root in any particular subtree of dependants) it becomes necessary to provide a hint to indicate that
the interfaces will participate in communication topologies that require the
presence of a stable unique identity.

To provide this hint while creating the package, in the *Additional Information*
section, when one adds inbound ROS topics/services/actions one can select the
***can be targeted***. This metadata is used by the platform to
bridge communications and enforce alias constraints.

![Can be targeted](/images/multi-robot-communication/can-be-targeted.png?classes=border,shadow&width=50pc)

![Inbound targeted](/images/multi-robot-communication/inbound-targeted.png?classes=border,shadow&width=50pc)

![Substitute](/images/multi-robot-communication/substitute.png?classes=border,shadow&width=50pc)

## Automatic Linking of ROS Interfaces
When a package declares a ROS interface like a ROS topic/service/action,
it is automatically made available to ROS nodes of other deployments
linked via the available design patterns like a dependent deployment.

Conside a sample composition such that two deployments:

* **C1** (corresponding to package **Q1**)
* **C2** (corresponding to package **Q2**)

that depend on a deployment **S** (corresponding
to package **P**). The user would quickly recognize that **C1**
and **C2** are effectively sibling deployments
with respect to a parent deployment **S**.

In this scenario, the following ROS specific interfaces are defined by
the packages **Q1** and **Q2** corresponding to **C1** and **C2**
respectively.

* **Q1** defines a ROS topic ***/test_topic***
* **Q2** defines a scoped ROS service ***/test_service*** (effectively available
  to peers as ***/C2/test_service***) and a ROS topic **/test_topic2**
* **P** defines inbound ROS interfaces for topic **/test_topic** and service
  **/test_service**

This set up is illustrated as shown below.
![auto linking of ROS interfaces](/images/dev-guide/manage-software-lifecycle/comm-topologies/auto-link-ros-interfaces.png?classes=border,shadow&width=50pc)

In such circumstances, availability of a topic/service/action
is dictated by the inbound interfaces defined by the package
**P**. In the above example, only ***/test_topic*** and
***/test_service*** (seen as ***/C2/test_service***) are available
to ROS nodes in the deployment **S**, while ***/test_topic2*** will not.

A key side effect is all of the deployments that depend on **S**
(**C1** and **C2**) will also have available an identical configuration
of ROS topics/services/actions to their ROS nodes. In this case,
***/test_topic*** and ***/test_service***
(seen as ***/C2/test_service***).

{{% notice note %}}
**Special case**: when package **P** is the publicly provided
**Rapyuta IO Local Communication Broker** package, the package is
then equivalent to a package with an Allow All inbound ROS
interface configuration. This implies all of the ROS
topics/services/actions provided by any child deployments
are available to all dependent siblings. In the above example,
***/test_topic***, ***/test_topic2*** and ***/test_service***
(seen as ***/C2/test_service***) are available
to both **C1** and **C2** (and any other siblings).
{{% /notice %}}

{{% notice note %}}
rapyuta.io makes ROS topics/actions/services available to the
*rosgraph* in the target deployment based on the composition.
However, data does not flow unless a ROS node within a particular
deployment tries to consume it by subscribing to a topic or
performing a service request.
{{% /notice %}}

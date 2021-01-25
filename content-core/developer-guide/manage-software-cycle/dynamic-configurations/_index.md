---
title: "Dynamic Configurations"
description:
type: developer-guide
date: 2019-10-30T15:04:21+05:30
pre: "1. "
weight: 401
---

The behavior of a robot (device) is determined by a set of
parameters, for instance the robot's velocity, controller gain threshold, or an image of a warehouse layout. 
These parameters are usually a consequence of the software and hardware used in the robot and the environment.

As a robotic developer, you need to represent, store, and review parameters. Additionally, you might want to access
parameters in your source code, modify a subset of parameters for a particular robot, or add new parameters and apply those to a
group of robots. A common pattern found in the robotics developer community is to arrive at a list of base parameters that are required for the application to run. When moving from the developer testbed to real-world operations, the operators, developers, and vendors
often wish to override the base parameters to satisfy various requirements.

The *rapyuta.io* platform provides a mechanism that allows a developer to set, review, update and override configuration for any connected robot. 
Configuration parameters in the ***rapyuta.io*** platform are represented by a *tree-like* hierarchical structure 
called  ***configuration hierarchy***.  



Every configuration hierarchy consists of the following four kinds of nodes:

- [RootNode](#rootnode)
- [AttributeNode](#attributenode)
- [ValueNode](#valuenode)
- [FileNode](#filenode)

 Consider the sample
configuration hierarchy, ***example***, as shown in the following figure.
The parameters of ***example*** are arranged in a specific order. The
hierarchy allows you to override the default parameters(or base parameters)
or extend existing parameters based on the country of operation.


To view an example of how you may leverage configuration hierarchies please refer to the illustrated section on [overriding behavior](resove-configuration-hierarchy/) 
 
![example configuration](/images/core-concepts/configurations/example-config.png?classes=border,shadow&width=20pc)

#### RootNode
The RootNode is the root of the configuration hierarchy tree. There
is only one RootNode per configuration hierarchy. Its name is
same as the name you provide while creating the configuration
hierarchy. The RootNode allows you to add an attribute, add/upload
 configuration files, and apply the base label configuration parameters
  to a device.

 It might contain multiple file nodes and exactly one
attribute node.

In this case, the RootNode is consequently ***example***.
![root node](/images/core-concepts/configurations/root-node.png?classes=border,shadow&width=20pc)

#### AttributeNode
Devices in rapyuta.io allow the user to set arbitrary labels (modeled as key value pairs) corresponding to attributes of an onboarded device such as vendor, warehouse, country of operation, software version etc.

The developer might need to override certain configuration based on one particular attribute of the device (eg device *country*).
To do this, the developer must create an  **AttributeNode** which effectively creates a new overriding *"branch"* in the *configuration hierarchy*. 
Each attribute node should have at lease one value node. Furthermore, you can also create more AttributeNodes 
from the corresponding value nodes and define the hierarchy of the configuration tree.

For example,
the ***country*** attribute splits the hierarchy subtree based on
the value of its children, such as ***USA*** and ***Japan*** (represented as
value nodes). rapyuta.io represents these distinctions as key-value pairs to
allow developers arbitrary flexibility in how they define their hierarchies. 

AttributeNodes can contain only value nodes, each corresponding to a
branch that you wish to represent. In the case of ***example*** 
configuration, attributes are ***country***, ***motor_controller***.
![attribute node](/images/core-concepts/configurations/attribute-nodes.png?classes=border,shadow&width=20pc)

#### ValueNode
Each value node corresponds to one particular value that is of the type of the parent attribute node (eg device *country=**Japan***)  corresponding to a particular branch of the configuration hierarchy that will be used to override the more general configuration values.
The parent attribute and the corresponding child  value nodes are of same color. It can contain
multiple file nodes and only one attribute node. Furthermore, from the attribute node, 
you can create multiple value nodes and define the hierarchy of the configuration tree.

In ***example*** configuration, ***USA*** and ***Japan*** are value nodes of **country** attribute.
![value node](/images/core-concepts/configurations/value-node.png?classes=border,shadow&width=20pc)

#### FileNode
A FileNode holds the actual configuration file containing parameters and data to be applied to the device. 
*rapyuta.io* configuration parameters allow the user to use any arbitrary file format (eg:images,xml,json,binary) while providing *special overriding behavior* for the **YAML** format.

FileNodes from a *sub-tree* (a more specific set of attributes and values vis a
vis its parent) that **recursively override/extend** the parameters defined in
any less specific FileNode with identical names. Subtrees may define
entirely new FileNodes (with different names), which are used in
resolution at that level.

In ***example*** configuration, ***example/sample.yaml***, ***USA/sample.yaml***
and ***Japan/sample.yaml*** are FileNodes such that the last two files may
either override or extend the existing ***example/sample.yaml*** file.
![file nodes](/images/core-concepts/configurations/parameters-files.png?classes=border,shadow&width=20pc)


To view an example of how you may leverage configuration hierarchies please refer to the illustrated section on [overriding behavior](resove-configuration-hierarchy/) 

---
title: "Resolving Configuration Hierarchy for a Device"
description:
type: developer-guide
date: 2019-10-30T15:06:45+05:30
pre: "1.1. "
weight: 403
---

A common pattern in the robotics developer community is to
arrive at a list of base parameters those are required for the
application to run. When moving from the developer testbed to
real-world operations, the operators, developers, and vendors
often wish to override or add new parameter values to the 
base parameters to satisfy various
requirements.

When you apply a configuration to a robot, rapyuta.io
utilizes the device labels to traverse the configuration
hierarchy. While the platform provides simple override rules for binary files, it provides a much cleverer approach for YAML type files commonly used in *roslaunch* configurations where it merges data from within the file as it traverses a hierarchy.

> Let's look at a practical example. It is likely you only want to
override a subset of parameters amongst numerous parameters defined
for a robot based on a particular criterion, for example, the velocity of an
AGV defaults to 5m/s, but the regulation in Japan requires you to
limit its velocity to 3m/s. For an AGV to be deployed in Japan,
you override its default velocity (5m/s) to 3m/s. Any other AGV
deployed outside of Japan might still require the default velocity of 5m/s.

> Another example of using the configuration parameter is the map images for the robots stationed at different warehouses. The map image for a robot "A" stationed at japan warehouse might be different than the map image of the same type of robot stationed at USA warehouse. ***rapyuta.io*** allows you to upload different map images as the configuration hierarchy to suit your need.

## Overriding configuration parameters

### Overriding Rules for FileNodes (Binary)

If you have uploaded the base parameters (or default parameters) as a binary file, and uploaded a binary file  with *different* content but the same file name at a *different level* but more specific level in the configuration hierarchy, when applying the configuration at the attribute level, the base parameter file is **replaced** with the file with the more specific attribute level.

In the following images, the checksum file for the base parameters file and the configuration file at the attribute level  **test.html** are different.
![base binary parameter](/images/core-concepts/configurations/root-binary-file.png?classes=border,shadow&width=65pc)

When applying the configuration to the devices, the base parameter file is replaced by the file you had uploaded at the attribute level. The following image displays the configuration file that is applied to the device.
![base binary parameter](/images/core-concepts/configurations/updated-binary-file.png?classes=border,shadow&width=65pc)

### Overriding Rules for FileNodes of type YAML

In the case of YAML files, while the order of resolution and overriding remains identical to binary files, the result is a little different.
The more specific FileNode content **does NOT replace** the content of the original file but **merges and replaces specific key value pairs within the YAML file**.  

The base parameters (or parameters defaults) file is usually located at
the root of the configuration hierarchy. In ***example*** configuration,
***example/sample.yaml*** file is the base parameters file, and it is located
under the ***example*** root node.

![parameter defaults](/images/core-concepts/configurations/parameter-defaults.png?classes=border,shadow&width=50pc)

The parameters are represented as **key: value** pairs like
***max_velocity: 5***, which indicates that the maximum velocity of an
AGV is 5m/s.

You can override or extend base parameters by defining **sample.yaml**
files at different levels of the configuration hierarchy.

Suppose that the regulation in Japan requires you to limit the
maximum velocity of an AGV from **5m/s** to **3m/s**. You can override the
***max_velocity*** of the AGV by assigning a new value to it. The
***sample.yaml*** file under the ***Japan*** value includes only the
***max_velocity*** parameter, but with its default overridden.

![override parameters](/images/core-concepts/configurations/override-max-vel.png?classes=border,shadow&width=65pc)

The final parameters file is a result of merging the base parameters
(***example/sample.yaml***) and the overridden parameters
(***Japan/sample.yaml***).



## Extending configuration parameters
You can add new parameters to extend the list of base
parameters. For instance, the ***USA/sample.yaml*** defines an additional parameter, ***example_param_usa: val***.

The resultant file after merging the base parameters in ***example/sample.yaml***
and newly added parameters in ***USA/sample.yaml*** will include
***example_param_val*** in addition to those already present.

![extend parameters](/images/core-concepts/configurations/extend-params.png?classes=border,shadow&width=80pc)


{{% notice info %}}
You may ***clone an existing configuration*** into another project.
Cloning saves you from the redundant task of defining the
same configurations from scratch again. However, cloning a configuration
inside the same project is not supported.
{{% /notice %}}

{{% notice info %}}
You may ***rename an already defined configuration***.
{{% /notice %}}
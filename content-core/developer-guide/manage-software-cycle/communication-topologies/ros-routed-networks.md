---
title: "ROS Routed Networks"
description:
type: developer-guide
date: 2019-10-25T12:54:26+05:30
pre: "c. "
weight: 436
---

Routed network is a rapyuta.io resource to enable ROS communication between different ROS package deployments. Binding a routed network resource to your deployment will enable other deployments on the same routed network to consume ROS topics/services/actions as defined in the package. 
Data flow occurs only when another package chooses to subscribe to a topic, call a service or call an action. 

#### Illustarted Example
For the purpose of this illustration lets assume the following network and packages.

* You have a routed network __networkN__
* You have __PackageA__ publishing _“topicA”_
* You have __PackageB__ publishing _“serviceB”_

We deploy the packages described above in the following configuration.

* Deploy __PackageA__ binding to routed network __networkN__ as _DeploymentA_
* Deploy __PackageB__ binding to routed network __networkN__ as _DeploymentB_

The result is as follows 

* ROS nodes in  _DeploymentA_ can now call _“serviceB”_
* ROS nodes in  _DeploymentB_ can now subscribe to _“topicA”_

A routed network can be deployed to a cloud or to a device.


## Cloud Routed Network

When a user deploys a routed network to the cloud it is considered a cloud routed network. Any compute resources (cpu/memory) consumed by this routed network deployment count against your cloud deployment hours quota.

Package deployments in the cloud __OR__ device can bind to a cloud routed network.


## Device Routed Network

In certain cases where communication is latency sensitive or has high throughput the user can choose to deploy a routed network to a device. 
While avoiding a round trip of information to the cloud minimizes latency and allows for better throughput __ONLY__ deployments on devices on the same local area network can bind to it. 



#### Routed networks can be deployed to a device with the following parameters:

* __Device:__ Any online device with docker runtime and AMD64 architecture
* __Device IP Interface:__ network interface (i.e., an IP address) that will be used by other deployments for communication to this routed network.
* __Restart policy:__ Kindly refer to the [restart policy](/developer-guide/manage-software-cycle/deployments/#restart-policy).

{{% notice tip %}}
On reboot devices configured using DHCP may boot up with a new IP address. This will cause the network configuration of a routed network deployed on it to become invalid. This can be avoided by assigning a static IP  to the device you intend to deploy a routed network to esp in production systems.
{{% /notice %}}

{{% notice info %}}
Follow this walkthrough to create and use a [routed network](/build-solutions/sample-walkthroughs/routed-network). 
{{% /notice %}}
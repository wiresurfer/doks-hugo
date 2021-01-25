---
title: "Managing Machines"
description:
type: developer-guide
date: 2019-10-24T11:39:53+05:30
pre: "II. "
weight: 225
---

The rapyuta.io platform attempts to bring all the various parts of your robotics solution under one roof. The unified tooling, API, and workflows strive to maintain uniformity in process and capabilities enabling the developer to focus less on managing distributed infrastructure and more
on the solution.

To make this feasible rapyuta.io implements device management capabilities into the core of the platform.

The following sections explain how to connect, interact, and use robots and devices on the platform.

## Architecture
The following image illustrates the architecture of device
manager system.
![Architecture](/images/core-concepts/device-management/architecture.png?classes=border,shadow&width=60pc)

## Device management in rapyuta.io

The device management layer in rapyuta.io is a critical part of the base infrastructure and services to enable core functionality.

The following sections are a non-exhaustive set of some of its responsibilities and features.

### Communication
The device manager service uses a server-agent communication
model where the *device manager* is the server-side component,
and the *device manager agent* is the agent component of the
model.

#### Network Address Translator (NAT) Traversal
Since the device manager agent initiates the connection to
the device manager, the communication works behind a firewall
and NAT network. You do not need to open any incoming ports on
your firewall.

#### Management Communication
rapyuta.io provides low throughput channels: control and
telemetry channels. These channels are activated when the device is
powered-on; connected to the internet, and registered with rapyuta.io
The channels are optimized for reliability, security for small
payloads like configuration commands, metrics and logs, and for
minimum utilization of resources. 

* **Control channel**    
  This channel provides a persistent connection between the device
  manager service and device manager agent. The service communicates
  with the agent by using the publish-subscribe pattern.
  The device manager service sends control commands to and receives
  event messages from device manager agent. The channel secures each
  device with a unique rotating AES key-based encryption.
* **Telemetry channel**     
  The registered device generates metrics and logs data. The device
  manager agent sends this data to the device manager service
  through the telemetry channel. The channel uses TLS encryption
  to secure each device together with a unique token.

{{% notice note %}}
Application communication is handled in diffrent channels. [Learn more](/developer-guide/manage-software-cycle/communication-topologies/) with a dedicated set of [features for ROS users](/developer-guide/manage-software-cycle/communication-topologies/ros-support/)
{{% /notice %}}

### Remote Debugging
The device manager lets you access a device remotely over a
Wide Area Network (WAN) connection. You may open an
interactive shell to the device through a web browser
WebSSH, and send individual shell commands to be executed
on the device.

### Remote Configuration
Dynamic configurations allow you to  control a device or a
group of devices based on a large set of configuration
parameters.

### Remote Monitoring
Remote monitoring provides better visibility into devices
regardless of where they are located. The device manager
service enables you to collect and visualize execution metrics
like CPU usage, memory usage, battery level, etc. of the device.

### Security 
The device manager ensures that the data
(telemetry and control) exchanged between
various components are encrypted.

* **End to end encryption**    
  On successful registration of a device with the device
  manager, all further communication between the server and
  the agent is encrypted using public-key cryptography.
* **Identity validation**    
  All devices that are registered with the device manager
  will possess unique credentials that are visible only to
  you. These credentials are used to identify devices across
  communication channels.

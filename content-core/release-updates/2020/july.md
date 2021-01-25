---
title: "July"
type: release-updates
date: 2020-07-10T16:18:22+05:30
weight: 834
---

## July 11

### Updates

The rapyuta.io docker runtime now uses the hostname of the device as the ROS_MASTER_URI. This will allow you to open rviz or echo topics on robots from the local network. All old devices will be updated on reboot.    
Running deployments will continue to work. 
{{% notice info %}}
Ubuntu(16.04 and 18.04) by default resolves the hostname to localhost. If you do change this behaviour on the host OS, roscore **will** not be able to start. A simple way to check if roscore can be started is to do `nslookup $(hostname)` if it returns a DNS record you are probably good to go. 
{{% /notice %}}

## July 29
Welcome to the July 29, 2020 release of the rapyuta.io platform.

#### Updates
- Ability to add port ranges for internally exposed endpoints: [Port ranges](/developer-guide/create-software-packages/package-internals/#exposing-endpoints-externally)

#### SDK
- **rapyuta.io Python SDK [0.15.0](/developer-guide/tooling-automation/python-sdk/#installation) released** 
- Added network interface in routed network sdk method
- Other fixes and improvements

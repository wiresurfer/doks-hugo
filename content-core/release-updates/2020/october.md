---
title: "October"
type: release-updates
date: 2020-10-14T15:00:22+05:30
weight: 834
---


## October 21
Welcome to the October 21, 2020 release of the rapyuta.io platform. There
are significant updates in this release that we hope you will like.

#### Features
* **Docker Push/Pull Secret**

    The *rapyuta.io* platform allows you to push and save a docker image to a private registry that you have created by using either catkin or docker recipe for future usage. If you create a build by using a docker file and a private image is referenced, the platform uses a pull secret to fetch the image and create the build for you. For more information, [click here](/developer-guide/create-software-packages/builds/build-creation).

* **Cloud Bridge Error List**
    
    New deployment [error codes](/developer-guide/manage-software-cycle/deployments/#error-codes) are introduced due to cloudbridge failures. Also, the *rapyuta.io* platform allows you to view the cloud bridge warning count histogram graph and the status of cloud bridge and routed network on the deployment details page. For more information, [click here](/developer-guide/manage-software-cycle/deployments/#network-configuration-for-executables).

#### Improvements
	
- Minor bug fixes and improvements to rapyuta.io APIs.

#### Breaking changes
	
- While cloning a git repository, ensure that you provide the appropriate protocol (HTTP/HTTPS). The HTTP to HTTPS redirection does not work while cloning the repositories.

#### SDK
**rapyuta.io Python SDK [0.17.1](/developer-guide/tooling-automation/python-sdk/#installation) released** 

- Fixed *Client.get_static_route_by_name()* method. 

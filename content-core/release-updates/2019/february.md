---
title: "February"
description:
type: release-updates
date: 2019-10-30T17:52:55+05:30
weight: 895
---
## February 13
Welcome to the February 13, 2019 release of rapyuta.io platform. There are a
number of significant updates in this version that we hope you will like.

#### Feature
* Deployment logs : Historical and Streaming logs     
  You can now access and view deployment logs as either historical logs or live streaming logs. You may search the logs, filter the logs using regular expressions, view specific logs within a time range, and download logs.

#### Notable Fixes
* Ensure *rapyuta-agent* has higher priority in case of resource constraint devices so as to prevent OOM (Out of Memory) killing of *rapyuta-agent*
* Seamlessly switch a device's runtime between preinstalled runtime and docker compose runtime
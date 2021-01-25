---
title: "Logging"
description:
type: developer-guide
date: 2019-10-25T13:22:14+05:30
pre: "1. "
weight: 510
---
Logs are verbose text messages and are used for debugging and monitoring.
Logs are generated when rapyuta.io builds the source code of a git repository,
by a device and during the life cycle of deployment.

rapyuta.io collects and indexes log data. There are three types of logs produced:

1. [Build Logs](/developer-guide/tooling-automation/logging/build-logs/)
2. [Deployment Logs](/developer-guide/tooling-automation/logging/deployment-logs/)
3. [Device Logs](/developer-guide/tooling-automation/logging/device-logs/)

All types of logs are available only for seven days, after which they are
automatically destroyed.
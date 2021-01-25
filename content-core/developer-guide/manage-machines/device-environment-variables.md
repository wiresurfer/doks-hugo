---
title: "Device Environment Variables"
description:
type: developer-guide
date: 2020-05-15T11:04:37+05:30
pre: "6. "
weight: 266
---
Some environment variables are available on the device after its successful onboarding.
These are also available to the deployments on the device. These are:

*  **RIO_DEVICE_NAME_LAST_SEEN**: Stores the name of the device that was present at the time of onboarding. Please note that changes made to the device’s name through rapyuta.io UI or APIs will not reflect back on this environment variable.
*  **RIO_DEVICE_ID**: Stores the unique identifier of the device.
*  **RIO_PROJECT_ID**: Stores the unique identifier of the project in which the device was created.


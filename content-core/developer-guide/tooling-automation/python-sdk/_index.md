---
title: "Python SDK"
description:
type: developer-guide
date: 2019-10-25T13:38:33+05:30
pre: "4. "
weight: 545
---
rapyuta.io Python SDK enables you to provision packages
either on the cloud or on a device, add a dependent
deployment, and access various rapyuta.io resources and
services in your python applications.

## Installation
It is recommended you install the latest Python SDK
using ***pip*** (most popular tool for installing
python packages).
```bash
pip install https://storage.googleapis.com/rio-sdk-python/rapyuta_io-0.19.0-py2.py3-none-any.whl
```
If you are using an old version of the Python SDK, please upgrade to the latest
[0.19.0](https://storage.googleapis.com/rio-sdk-python/rapyuta_io-0.19.0-py2.py3-none-any.whl)
version.



## Requirements
The rapyuta.io Python SDK is compatible with python
applications supporting Python2.7.

{{% notice info %}}
We have added Python3.9 support to the rapyuta.io python SDK. Note that this is still in alpha stage.
{{% /notice %}}

## SDK reference
If you are looking for more detailed information about any class in the SDK, feel
free to consult the [full SDK reference](https://sdkdocs.apps.rapyuta.io/).

## Examples
The following example walkthroughs will help you get
started quickly with rapyuta.io Python SDK:

1. [Basic Publisher Subscriber](/developer-guide/tooling-automation/python-sdk/sample-walkthroughs/basic-pubsub/)
2. [Deployment Composition](/developer-guide/tooling-automation/python-sdk/sample-walkthroughs/deployment-composition/)

{{% notice info %}}
Before walking through the examples, ensure you [obtain the auth token, project ID, device ID, package ID and plan ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/).
{{% /notice %}}
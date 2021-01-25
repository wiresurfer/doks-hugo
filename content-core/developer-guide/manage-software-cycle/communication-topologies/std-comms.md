---
title: "Standard Web Protocols"
description:
type: developer-guide
date: 2019-10-25T12:53:52+05:30
pre: "a. "
weight: 431
---
rapyuta.io facilitates the communication for common protocols such as HTTP, WebSocket, TLS with the help of *Link Injection* between the dependent deployments

{{% notice note %}}
Link injection is the mechanism rapyuta.io relies on for composition features corresponding to exposed network endpoints.
{{% /notice %}}

### Link Injection

For instance if you have a deployment *P* running on rapyuta.io that exposes a network endpoint defined as **SAMPLE_ENDPOINT** with the URL address: *https://inst-awesomesauce-url.apps.rapyuta.io:443* 
the rapyuta.io platform can use the above URL to determine the corresponding **HOST** and **PORT** values as follows:

* **HOST**: *inst-awesomesauce-url.apps.rapyuta.io*
* **PORT**: *443*

rapyuta.io can now make this information available to any other resource it manages like deployments.

Consider another deployment, for instance, *C* such that user deploying *C* selects *P* as a ***dependent deployment***. Now for the purpose of linking deployments a parent-child relationship is established between deployments *P* (the parent) and *C* (the child). 

rapyuta.io will make exposed endpoint information available to *C* (the child) by constructing and injecting environment variables corresponding to each endpoint exposed by *P* (the parent) using the following rule.

* **\<ENDPOINT_NAME\>**
* **\<ENDPOINT_NAME\>_HOST**
* **\<ENDPOINT_NAME\>_PORT**

Drawing from the aforementioned example this would correspond to following evnironment variables and their corresponding values: 

* **SAMPLE_ENDPOINT** : *https://inst-awesomesauce-url.apps.rapyuta.io:443* 
* **SAMPLE_ENDPOINT_HOST** : *inst-awesomesauce-url.apps.rapyuta.io*
* **SAMPLE_ENDPOINT_PORT** : *443*

The developer of the package running in *C* can now access these environment variables in code.

For example, the developer would access the value of the **SAMPLE_ENDPOINT** in a Python application using
```python
import os
os.getenv('SAMPLE_ENDPOINT')
os.getenv('SAMPLE_ENDPOINT_HOST')
os.getenv('SAMPLE_ENDPOINT_PORT')
```

### Self Injection
rapyuta.io injects network endpoints as environment variables
during the deployment phase.

Suppose that a package defines a network endpoint, **SAMPLE_INTERFACE_POINT**, which is externally exposed.
The port and target port are set to 443 and 5000, respectively.
When the package is deployed, rapyuta.io injects **SAMPLE_INTERFACE_POINT**
as an environment variable. You can access all of the environment
variables in a deployment via the **Shell Access** option.

![Example network endpoint](/images/chapters/developer-guide/create-software-pkgs/pkg-internals/sample-enp.png?classes=border,shadow&width=40pc)

Click on **Shell Access** > **SSH** to open a Linux terminal
of the deployment. Enter the following commands the network endpoint,
its host URL address, and port.

```bash
echo $SAMPLE_INTERFACE_POINT
```

```bash
echo $SAMPLE_INTERFACE_POINT_HOST
```

```bash
echo $SAMPLE_INTERFACE_POINT_PORT
```

![Network endpoint](/images/chapters/developer-guide/create-software-pkgs/pkg-internals/endpoint-env-var.png?classes=border,shadow&width=40pc)
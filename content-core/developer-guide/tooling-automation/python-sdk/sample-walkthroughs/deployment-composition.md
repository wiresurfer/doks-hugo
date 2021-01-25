---
title: "Deployment Composition"
description:
type: developer-guide
date: 2019-10-25T13:54:24+05:30
pre: "2. "
weight: 565
---
## Learning objectives

1. Set up block storage for an application.
2. Add configuration parameters like secret and access
   keys to application.
3. Programmatically use
[rapyuta.io Python SDK](/developer-guide/tooling-automation/python-sdk/) in Python applications.

## Prerequisites

1. [Install rapyuta.io SDK](/developer-guide/tooling-automation/python-sdk/#installation) in development environment.
2. Learn how to obtain
   1. [authorization token](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#auth-token)
   2. [project ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#project-id)
   3. [package ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#package-id)
   4. [plan ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#plan-id)
3. Follow the [object store deployment walkthrough](/developer-guide/create-software-packages/persistent-storage/obj-store-deployment-tutorial/).

## Difficulty
Intermediate

## Estimated time
15 minutes

## Assumptions

1. ***Volume Storage*** is the persistent volume that will be added to the file server application.
2. ***PROJECT_ID*** is a unique identification value of the project in which the package, **MinIO File Server**, is created. It is of type *string*.
3. The package is ***MinIO File Server*** with package ID ***PACKAGE_ID***, which is of type *string*.
4. ***MinIO_FS*** is a component of the package, ***MinIO File Server***. 
5. ***PLAN_ID*** is the plan ID of ***MinIO File Server*** package. It is of type *string*. 
6. ***AUTH_TOKEN*** is the authorization token for accessing rapyuta.io resources and services. It is of type *string*.

### Create MinIO File Server Package
Create a package called **MinIO File Server** on the rapyuta.io platform.
A [step-wise guide](/developer-guide/create-software-packages/persistent-storage/obj-store-deployment-tutorial/) for creating the package is detailed in the prerequisite walkthrough.

### Code Walkthrough
Firstly, you must authenticate for accessing rapyuta.io services
and resources from your Python application.

```python
# Authorization code snippet
from rapyuta_io import Client, DiskType

client = Client(AUTH_TOKEN, PROJECT_ID)
```

Deploy the persistent volume, which is provided by rapyuta.io, with ***32GiB*** capacity and ***SSD*** disk type.

```python
## Define instance of persistent volume
volume = client.get_persistent_volume()
storage_volume = volume.create_volume_instance("Volume Storage", 32, DiskType.SSD)
storage_volume.poll_deployment_till_ready()
```

Mount the volume on the **MinIO_FS** component of **MinIO File Server** package at the ***/data*** mount point.

```python
# Mount volume on the file server application
minio_file_server = client.get_package(PACKAGE_ID)
pkg_provision_config = minio_file_server.get_provision_configuration(PACKAGE_PLAN_ID)
pkg_provision_config.mount_volume(component_name="MinIO_FS",
                                  volume_instance=storage_volume, mount_path="/data")
```
Define secret and access keys for signing in to MinIO
file server application at the externally exposed
endpoint **FileStorage**.

```python
# Define secret and access keys for file server
pkg_provision_config.add_parameter("MinIO_FS", "MINIO_SECRET_KEY", "secretphrase")
pkg_provision_config.add_parameter("MinIO_FS", "MINIO_ACCESS_KEY", "accesskey")
```

Deploy **MinIO File Server** with the added persistent
volume **Volume Storage** on the cloud. The corresponding deployment is called
**File Storage With Data Permanence**.

```python
# Deploy file server with storage volume
volume_powered_deployment = minio_file_server.provision(deployment_name="File Storage With Data Permanence",
                                                        provision_configuration=pkg_provision_config)
volume_powered_deployment.poll_deployment_till_ready(retry_count=10)
print volume_powered_deployment.get_status()
```

{{% notice info %}}
Adding a persistent volume to a file server application enables
it to preserve data files saved on it even if the application
is stopped and deployed again.
{{% /notice %}}

Put together the above code snippets in a single file, ***deployment-composition.py***, save the program and close the file.

```python
# deployment-composition.py


from rapyuta_io import Client, DiskType

# Authorisation code snippet
client = Client(AUTH_TOKEN, PROJECT_ID)

## Define instance of persistent volume
volume = client.get_persistent_volume()
storage_volume = volume.create_volume_instance("Volume Storage", 32, DiskType.SSD)
storage_volume.poll_deployment_till_ready()

# Mount volume on the file server application
minio_file_server = client.get_package(PACKAGE_ID)
pkg_provision_config = minio_file_server.get_provision_configuration(PACKAGE_PLAN_ID)
pkg_provision_config.mount_volume(component_name="MinIO_FS", volume_instance=storage_volume, mount_path="/data")

# Define secret and access keys for file server
pkg_provision_config.add_parameter("MinIO_FS", "MINIO_SECRET_KEY", "secretphrase")
pkg_provision_config.add_parameter("MinIO_FS", "MINIO_ACCESS_KEY", "accesskey")

# Deploy file server with storage volume
volume_powered_deployment = minio_file_server.provision(deployment_name="File Storage With Data Permanence",
                                                        provision_configuration=pkg_provision_config)
volume_powered_deployment.poll_deployment_till_ready(retry_count=10)
print volume_powered_deployment.get_status()
```

At the terminal prompt, run the program using the command:
```bash
$ python deployment-composition.py
```

The output is an object of the class
[***DeploymentStatus***](https://sdkdocs.apps.rapyuta.io/#rapyuta_io.clients.deployment.DeploymentStatus),
which contains values such as deployment ID, deployment name, deployment status, deployment phase, package identifier, etc.
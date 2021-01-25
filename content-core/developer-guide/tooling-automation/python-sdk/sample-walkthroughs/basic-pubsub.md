---
title: "Basic Publisher Subscriber"
description:
type: developer-guide
date: 2019-10-25T13:41:24+05:30
pre: "1. "
weight: 560
---
## Learning objectives
The walkthrough gives an overview of how to:

1. configure and provision a package
2. deploy a package on the cloud
3. deploy a package on a device
4. add a dependent deployment

programmatically using
[rapyuta.io Python SDK](/developer-guide/tooling-automation/python-sdk/) in your
python application.

## Prerequisites
1. Read the [developer guide](/developer-guide/)
   of rapyuta.io
2. [Install rapyuta.io SDK](/developer-guide/tooling-automation/python-sdk/#installation)
   in your development environment.
3. Learn how to obtain:
   1. [authorization token](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#auth-token)
   2. [project ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#project-id)
   3. [package ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#package-id)
   4. [plan ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#plan-id)
   5. [device ID](/developer-guide/tooling-automation/python-sdk/sdk-tokens-parameters/#device-id)

## Difficulty
Beginner

## Estimated time
15 minutes

## Assumptions
1. ***Talker*** is a ROS package created in rapyuta.io,
   and it behaves as a ROS publisher.
2. ***Listener*** is another ROS package created in
   rapyuta.io, and it behaves as a ROS subscriber. **LISTENER** is the
   component of the package that will be deployed on a device.
3. ***PROJECT_ID*** is a unique identification value
   of the project in which **Talker** and **Listener**
   packages are created. It is of type *string*.
4. ***TALKER_ID*** and ***LISTENER_ID*** are the
   package IDs of the ***Talker*** and ***Listener*** packages
   respectively. The values are of type *string*.
5. ***TALKER_PLAN_ID*** and ***LISTENER_PLAN_ID***
   are the plan IDs of the default plan of ***Talker*** and
   ***Listener*** packages, respectively. The values are of
   type *string*.
6. ***AUTH_TOKEN*** is the authorization token for accessing rapyuta.io 
   resources and services. Its value is of type *string*.
7. ***DEVICE ID*** is a unique identification value of a device on
   rapyuta.io, and it is of type *string*.


### Creating the **io-tutorial** build
 
To create the build, follow below steps. Skip the following steps if you have already created an *io-tutorials* build earlier.

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `io-tutorials`
4. In the Git repository box, enter the url address : `https://github.com/rapyuta/io_tutorials` 
and select **Build Recipe** as Catkin.
5. Go to the next step and click on next, the build will be created.

The build takes about two to five minutes to build the source code in the *io_tutorials* repository into a running docker container. You may analyze the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which helps in debugging failed builds.
Please proceed to creation of package once the build is Complete.

### Create Talker Package

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD PACKAGE**.
3. In the **Package Name** box, type in a name for the package like `Talker`.
4. In the **Package Verison** box, enter the version of the package
   you are creating. The default value is *1.0.0*
5. Make sure **Is singleton package** is ***not selected***.
6. Ensure **Is bindable package** is ***selected***.
7. In the **Description** box, explain what the package is about,
   for instance, the description of this package is `ROS Publisher`.
8. Click **NEXT**.
9.  In the **Component Name** box, enter a name for the component say `TALKER`.
10. Select **Cloud** for **Component Runtime**.
11. Ensure **Is ROS Component** is selected.
12. Select **Kinetic** for **ROS Version**.
13. The default value of **Replicas to run the component** is set to 1
14. In the **Executable Name** box, enter a name for an executable,
    say `talker_executable`.
15. Click **Builds** for **Executable Type**.
16. Select **io-tutorials** builds from the dropdown
17. In the **Command to run in the docker container** box, enter the command:
    `roslaunch talker talker.launch`
18. The ***talker_executable*** publishes a ROS topic `/telemetry`.
    To add a ROS topic, click **Add ROS topic**. In the **Name** box,
    type in the ROS topic. Select **Maximum** as the value for **QoS**.
19. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

### Create Listener Package

1. On the left navigation bar, click **CATALOG**.
2. Click **ADD PACKAGE**.
3. In the **Package Name** box, type in a name for the package
   like `Listener`.
4. In the **Package Version** box, enter the version of the package
   you are creating. The default value is *1.0.0*
5. Make sure **Is singleton package** is ***not selected***.
6. Ensure **Is bindable package** is ***selected***.
7. In the **Description** box, explain what the package is about,
   for instance, the description of this package is `ROS Subscriber`.
8. Click **NEXT**.
9.  In the **Component Name** box, enter a name for the component say `LISTENER`.
10. Select **Device** for **Component Runtime**.
11. Ensure **Is ROS Component** is selected.
12. Select **Kinetic** for **ROS Version**.
13. Set **Restart Policy** to **Never**.
14. In the **Executable Name** box, enter a name for an
    executable say `listener_executable`.
15. Set **Executable Type** to **Default**.
16. In the **Command to run in the docker container** box, enter
    the command: `roslaunch listener listener.launch`
17. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

### Code Walkthrough
Firstly, you need to authenticate so as to access rapyuta.io services from within
your python application.
```python
# Authentication
from rapyuta_io import Client

client = Client(AUTH_TOKEN, PROJECT_ID)
```

Retrieve the ***Talker*** package by its package ID, and then deploy
it on the cloud. The resulting deployment is called ***ROS PUBLISHER***.

```python
# Deploy Talker package on cloud
talker = client.get_package(TALKER_ID)
talker_configuration = talker.get_provision_configuration(TALKER_PLAN_ID)
talker_cloud_deployment = talker.provision(deployment_name="ROS PUBLISHER",
                                           provision_configuration=talker_configuration)
talker_cloud_deployment.poll_deployment_till_ready()
```

Similarly, deploy ***Listener*** package on the cloud.
Since the resulting ***ROS SUBSCRIBER*** deployment depends on ***ROS PUBLISHER***
deployment, add the later as a dependent deployment of the former.

```python
# Deploy Listener package on device
listener = client.get_package(LISTENER_ID)
listener_configuration = listener.get_provision_configuration(LISTENER_PLAN_ID)
device = client.get_device(DEVICE_ID)
listener_configuration.add_device("LISTENER", device)
listener_configuration.add_dependent_deployment(talker_cloud_deployment)
listener_device_deployment = listener.provision(deployment_name="ROS SUBSCRIBER",
                                                provision_configuration=listener_configuration)
listener_device_deployment.poll_deployment_till_ready()
```

Put the above code snippets together in a file, ***talker-listener.py***,
save the program and close the file.

```python
# talker-listener.py

from rapyuta_io import Client

# Authentication
client = Client(AUTH_TOKEN, PROJECT_ID)

# Deploy Talker on cloud
talker = client.get_package(TALKER_ID)
talker_configuration = talker.get_provision_configuration(TALKER_PLAN_ID)
talker_cloud_deployment = talker.provision(deployment_name="ROS PUBLISHER", provision_configuration=talker_configuration)
talker_cloud_deployment.poll_deployment_till_ready()

# Deploy Listener on device
listener = client.get_package(LISTENER_ID)
listener_configuration = listener.get_provision_configuration(LISTENER_PLAN_ID)
device = client.get_device(DEVICE_ID)
listener_configuration.add_device("LISTENER", device)
listener_configuration.add_dependent_deployment(talker_cloud_deployment)
listener_device_deployment = listener.provision(deployment_name="ROS SUBSCRIBER", provision_configuration=listener_configuration)
listener_device_deployment.poll_deployment_till_ready()

# Get status of ROS SUBSCRIBER deployment
print subscriber_deployment.get_status()
```

At the terminal prompt, run the program using the command:
```bash
$ python talker-listener.py
```

The output is an object of the class
[***DeploymentStatus***](https://sdkdocs.apps.rapyuta.io/#rapyuta_io.clients.deployment.DeploymentStatus),
which contains values such as:

* deployment ID
* deployment name
* deployment status
* deployment phase
* package ID and other details.

The final deployment is running successfully if the value of the *deployment
status* is ***Running***.

To verify if the program has executed correctly, click on the **Historical Logs**
tab of **ROS SUBSCRIBER** deployment to view the output as shown:
![logs of ROS SUBCRIBER](/images/python-sdk-images/basic-pubsub/talker-listener-log.png?classes=border,shadow&width=50pc)
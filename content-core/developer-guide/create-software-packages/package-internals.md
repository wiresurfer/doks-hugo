---
title: "Package Internals"
description:
type: developer-guide
date: 2019-10-25T12:34:08+05:30
pre: "2. "
weight: 275
---
### What is a package ?
A package is a fundamental rapyuta.io resource that represents a declaration of your application. A package is the smallest unit of deployment in rapyuta.io. It can be deployed either on a device or the cloud or both.

A package encapsulates information about what strategy is used to build it, its compatibility and runtime requirements, network endpoints and ROS interfaces it exposes, and any configuration information it may require.

Each package consists of components, which are made up of individual executables. 

{{% notice note %}}
 **Advanced users** of rapyuta.io should note a package internally supports multiple **plans**, each which in turn contains the necessary components.
  This feature is intended to facilitate complex usecases that require the developer to maintain the user to represent a slightly different configuration of a software package. *For more details <a href="#" onclick="javascript:FreshWidget.show();">contact support</a>.*
{{% /notice %}}

### Executables
Executables within a component are always executed on the same physical/virtual compute node and share a ROS Master (in the case of ROS applications).
An executable is a runnable entity such as:

* **rapyuta.io Builds**    
Executables can reference existing **rapyuta.io Build**. Builds help you create a GitOps pipeline by
specifying a repository and a build recipe.
rapyuta.io can then build your git source code into a container image.
Executables referencing builds use the generated images at the time of package deployment.
Custom bash command can additionally be specified and is executed when the package is deployed.


{{% notice info %}}
Executables referencing builds use docker images at the time of deployment,
Packages containing such executables can't be deployed on devices with a Preinstalled runtime
{{% /notice %}}

* **Docker image**    
A docker image is used as an executable. When a deployment is triggered, rapyuta.io
pulls a docker image from the docker registry. Additionally, you may specify a
bash shell command, which overrides the
[entry point](https://docs.docker.com/engine/reference/run/#cmd-default-command-or-options)
of the docker container.


{{% notice info %}}
The maximum size of the docker image is 10GB for cloud deployment. If your docker image is private, then you will need to [create a docker secret](/developer-guide/create-software-packages/secrets/docker-registry/#creating-a-docker-pull-secret) and provide it in credentials.
{{% /notice %}}



* **Bash command**    
A simple bash shell command is an executable. If you choose the **Executable Type** as **Default**, the bash shell command becomes an executable. In this case, the executable can run only on **Preinstalled device** runtime. rapyuta.io assumes that all dependencies that are required to run the command are already present on the device where
the command will execute.

### Components
A component is a set of executables. All executables are deployed in unison on
the desired [*Component Runtime*](/developer-guide/create-software-packages/package-internals/#component-runtime).
All executables of a component communicate via Inter-Process Communication (IPC). An executable listening on a port is accessible to its sibling executables via localhost.

{{% notice info %}}
The number of volumes attached to a component must be less than or equal to the sum of all the cpu cores of all executables of a component.
{{% /notice %}}

Mathematically,    
number of volumes attached to a component **<=** [floor](https://en.wikipedia.org/wiki/Floor_and_ceiling_functions)(sum of all the cpu cores of all executables in the component)

Suppose a component has two executables, **execA** and **execB**. The executables may have the following cpu core values:

1. **execA**: 1 cpu core, **execB**: 0.5 cpu core, number of volumes attached is 1
2. **execA**: 1 cpu core, **execB**: 2 cpu cores, number of volumes attached <= 3
3. **execA**: 0.5 cpu core, **execB**: 0.5 cpu core, number of volumes attached is 1

{{% notice note %}}
Components are further nested into **plans**. A rapyuta.io "package" may contain multiple plans, and each plan represents a different configuration of a package. At this point, when you add a new package in the rapyuta.io, there is always a single plan associated with the package. A plan is uniquely identified by its plan ID
{{% /notice %}}

### Component Runtime
A component of a package may be deployed either on the **cloud** or on a **device**.

When deployed on the cloud, the component has cloud runtime. Whereas, the component deployed on a device has device runtime.

### Configuration Parameters
{{% notice info %}}
configuration parameters operate at the level of component and apply to executables in the component only
{{% /notice %}}

In line with the 12-Factor application philosophy, rapyuta.io allows the package author to pass configuration as environment variables that may be consumed by executables running within a component.

These are mapped to environment variables made available to your code. They are modeled as key-value pairs (where both the key and the value are strings) accessible by the user’s code using standard environment variable look-up techniques provided by the programming language.

The package author can choose to provide default values. These values may be overridden by the user while deploying the package.

{{% notice note %}}
A **package** may choose to declare environment variables as exposed from within its constituent components allowing dependent deployments to receive these values during deployment binding phase. Refer to the section on [binding](/developer-guide/manage-software-cycle/compose-software/binding/) for more details
{{% /notice %}}

{{% notice note %}}
The platform injects environment variables corresponding to exposed parameters.
{{% /notice %}}

### Network Endpoints
{{% notice info %}}
Individual components of a package expose network endpoints, which are
defined by users.
{{% /notice %}}
Components, which are deployed on the cloud, may have network endpoints. A network endpoint is a combination of an IP address and a port number. The endpoints may or may not be exposed publicly.

When creating an endpoint, you must provide a name for the endpoint, select the desired network protocol, and specify a target port.

{{% notice info %}}
The name of a network endpoint must consist of alphabets, digits, or an underscore ( _ ) and must not begin with a digit.
{{% /notice %}}

**Port** is where the application's service is made visible to other services.

**Target port** is where the application needs to be listening for network requests for the service to work.

rapyuta.io injects network endpoints as environment variables during the deployment phase.

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

#### Exposing Endpoints Internally
You can restrict access to a network endpoint by ensuring that **Exposed externally** option is not selected.

The only protocol available is the **TCP** for which the value of the **Port** field is set to ***443*** by default. However, you can change the port's value.
![internal endpoint](/images/core-concepts/network-endpoints/internal-endpoint.png?classes=border,shadow&width=40pc)

You can also use port range for an endpoint by selecting **Port Range** toggle. A Port Range on an endpoint will allow you to open multiple ports on a single DNS hostname.
{{% notice info %}}
By default the Target Port is same as the Port.
{{% /notice %}}
{{% notice info %}}
A maximum 50 ports are allowed for an endpoint.
{{% /notice %}}
{{% notice info %}}
Allowed format is comma separated Port Ranges. Each Port Range is either a single port or a range of port mentioning the from port and to port separated by a hyphen (-). Examples: 5000 or 443-445 or 3446-3449,3500,3510-3530
{{% /notice %}}
![internal endpoint port range](/images/core-concepts/network-endpoints/internal-endpoint-port-range.png?classes=border,shadow&width=40pc)
#### Exposing Endpoints Externally
Select **Exposed externally** checkbox to expose a network endpoint publicly over the internet.

The supported protocols at their respective ports (cannot be modified) are:

* HTTP/Websocket exposed on port ***80***
* HTTPS/WSS exposed on port ***443***
* Secure TCP (TLS/SNI) exposed on port ***443***

The **Secure TCP (TLS/SNI)** protocol uses [SNI](https://en.wikipedia.org/wiki/Server_Name_Indication) headers for routing the request to the desired backend.
![external endpoint](/images/core-concepts/network-endpoints/external-endpoint.png?classes=border,shadow&width=40pc)

rapyuta.io generates a random URL/route that is exposed on the public internet for the required endpoint when the deployment is created. You can view the Fully Qualified Domain Name (FQDN) of an endpoint on the details page of deployments.

![FQDN of external endpoint](/images/tutorials/hello-world/network-endpoint.png?classes=border,shadow&width=50pc)

{{% notice info %}}
rapyuta.io injects environment variables corresponding to linked network endpoints during deployment binding phase. Refer to the section on [Link Injection](/developer-guide/manage-software-cycle/communication-topologies/std-comms/) for more details.
{{% /notice %}}

#### Exposing Endpoints with Static URL
To get a deterministic URL/route for your application while exposing the network endpoint externally, you must bind it to a static route.

rapyuta.io enables you to create a static route URL and give it a globally unique FQDN. When you add a static route, an externally exposed endpoint is essentially guaranteed to be available at the URL of that particular static route. It makes externally exposed endpoints (and hence the deployments exposing them) resilient to failure or re-deployment, facilitates maintenance and upgrades to the backend/deployment while retaining the same unique globally available URL.

To create a static route:

1. On the left navigation bar, click **STATIC ROUTES**.
2. Click **ADD NEW STATIC ROUTE**.
3. Enter a name for **Static Route URL**.
4. Click **CONTINUE**.

Observe that the name of the static route will be a subdomain belonging to ***.ep-r.io*** (essentially the provided name will be suffixed with ***.ep-r.io*** to form the FQDN). For instance, if the name of the static route is ***my-example-server***, the static route URL will be ***my-example-server.ep-r.io***

![Create static route](/images/dev-guide/create-software-pkgs/pkg-internals/static-routes/create-sr.png?classes=border,shadow&width=40pc)

{{% notice info %}}
The name of a static route has lowercase alphanumeric characters, or a hyphen, and must begin and end with an alphanumeric character, and must not be certain keywords, and it must be at least 4 characters and less than 64 characters long.
{{% /notice %}}

{{% notice note %}}
Once created, you cannot edit the name of a static route.
{{% /notice %}}

To bind a static route to an externally exposed endpoint, which is defined in a package, during the deployment process:

1. Click **Add Static Route**.
![Add static route](/images/dev-guide/create-software-pkgs/pkg-internals/static-routes/add-sr.png?classes=border,shadow&width=40pc)
2. Select an external network endpoint from the drop-down list.
3. Select a static route from the drop-down list.
![Select endpoint static route pair](/images/dev-guide/create-software-pkgs/pkg-internals/static-routes/selection.png?classes=border,shadow&width=40pc)

It creates a mapping between an external network endpoint and a static route. You can unbind a static route from a network endpoint by clicking on the delete icon. In this example, the static route ***my-example-server*** is bound to the network endpoint ***server_endpoint*** as shown below:

![Bind static route](/images/dev-guide/create-software-pkgs/pkg-internals/static-routes/mapping-bind-sr.png?classes=border,shadow&width=40pc)

On deploying the package after binding a static route, the network endpoint URL address becomes deterministic and is a constant. It implies that even if the deployment is stopped and provisioned again with the same static route, the network endpoint URL address remains the same.

A package deployment can have multiple static routes. However, a single static route is used for a single deployment.

A static route is ***globally unique*** across the rapyuta.io platform.

{{% notice note %}}
Refer to [billing and usage](/pricing-support/pricing/billing-usage) to understand the limits applied on static routes for different subscription plans.
{{% /notice %}}





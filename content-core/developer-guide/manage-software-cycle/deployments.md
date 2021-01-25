---
title: "Deployments"
description:
type: developer-guide
date: 2019-10-25T12:49:01+05:30
pre: "2. "
weight: 405
---
A deployment is a rapyuta.io resource that represents a unique
instantiation of a rapyuta.io package. It holds information
about the package deployed, the configuration used and interfaces
exposed. It possesses a unique identifier and provides a mechanism
to introspect its phase and state that are needed to ascertain
the state of a system.

Tooling such as logs, debug terminals and other automation leverage
this uniquely identifiable resource to allow the operator to manage,
debug and observe a particular running instance of their application.

Deployments may support linking and binding to allow the user to
combine multiple different applications to help realize
a potentially complex robotics solution.

The deployment object exposes information to help you introspect
the state of the system and set policies on mechanisms that
attempt recovery of the desired state on degradation. 

The following sections let the developer learn more about

- [Phases](#phases)
- [Status](#status)
- [Error Codes](#error-codes)
- [Restart Policy](#restart-policy)
- [Network Configuration for Executables](#network-configuration-for-executables)
- [Deploying A Package](#deploying-a-package)
- [Update/Re-Deploy A Deployment in-place](#update-re-deploy-in-place)

## Phases
The lifecycle of a deployment consists of multiple phases. The **DEPLOYMENT PHASE**
indicates the current phase of the deployment in its lifecycle.

The below table lists the phases of deployment as they appear in the lifecycle:


| Deployment phase | Description |
| ---------------- | ----------- |
| In progress | accepts request to deploy a package and starts deployment process |
| Provisioning | pulls a docker image and creates a running instance of the image (docker container) for each executable of the component |
| Succeeded | each executable of every component is successfully started |
| Failed to start | error occurred during In progress phase |
| Partially deprovisioned | you deprovisioned a deployment, but there is at least one component that could not be deprovisioned |
| Deployment stopped | you deprovisioned a deployment, and all of its components are stopped |
| Failed To Update | One or more component failed while updating the deployment |

![Deployment](/images/core-concepts/deployments/deployment-phase.png?classes=border,shadow&width=60pc)

## Status
rapyuta.io enables you to monitor the current status of each executable of a
component that is deployed. The status of deployment
depends on the combined status of all components participating in the deployment.

The following table lists the statuses you may see during the **Provisioning**
deployment phase:


| Status | Description |
| ------ | ----------- |
| Pending | docker image is being pulled, or docker container is being created |
| Error | error occurs while pulling a docker image or creating a docker container |

The following table lists the statuses you may see during the **Succeeded**
deployment phase:

| Status | Description |
| ------ | ----------- |
| Running | executables of components are running |
| Pending | restarting executable due to runtime error in the application or rapyuta.io software |
| Error | runtime error occurred |
| Unknown | rapyuta.io is unaware of the current status |

If the status of an executable reads **Pending** or **Error**, you
are provided the cause of the status as **Reason**.

![Reason field](/images/core-concepts/deployments/reason-field.png?classes=border,shadow&width=50pc)

## Error Codes
If the overall deployment **STATUS** is **Error**, rapyuta.io
displays an error code along with a brief description of the error.
The following table lists available error codes, short descriptions
and the recommendations you should take:

| Error code | Description | Recommended action |
| ---------- | ----------- | ------------------ |
| DEP_E151 | device is either offline or not reachable | check the internet connection of the device |
| DEP_E152 | executables of the component deployed on the device either exited too early or failed | in the docker deployments this may indicate an error with the entrypoint or command, verify and fix it |
| DEP_E153 | unable to either pull the docker image or build the source code for the component deployed on cloud | verify that the docker image provided while adding the package still exists at the specified registry endpoint |
| DEP_E154 | executables of the component deployed on cloud exited too early | troubleshoot the failed component by analyzing deployment logs |
| DEP_E155 | executables of the component deployed on cloud failed | troubleshoot the failed component by analyzing deployment logs |
| DEP_E156 | dependent deployment is in error state | troubleshoot the dependent deployment that is in error state |
| DEP_E161 | docker image not found for executables of components deployed on device | verify that the path of the docker image is valid |
| DEP_E162 | Validation error. Cases include:<ul><li>Inconsistent values of ROS distro and CPU architecture variables for the device and package being provisioned.</li><li>rapyuta.io docker images not present on docker device.</li></ul> | <ul><li>Create package with appropriate values for ROS distro and CPU architecture variables.</li><li>Onboard the device again.</li></ul> |
| DEP_E163 | application has stopped and exited unexpectedly, and crashes continuously | debug the application using the corresponding deployment logs |
| DEP_E171 | cloud bridge encountered duplicate alias on the device. | change the alias name during deployment and ensure that there is no duplication of  alias name under the same routed network. For more information about alias, [click here.](/developer-guide/manage-software-cycle/communication-topologies/ros-support/#ros-environment-aliases-runtime-identity-assignment)</a> |
| DEP_E172 | compression library required for the cloud bridge is missing on the device. | re-onboard the device.</a> |
| DEP_E173 | transport libraries required for the cloud bridge is missing on the device. | re-onboard the device.</a> |
| DEP_E174 | cloud bridge on the device encountered multiple ROS service origins. | do not add multiple deployments with the same ROS service endpoint under the same routed network.</a> |
| DEP_E175 | python actionlib/msgs required for the cloud bridge is missing on the device. | re-onboard the device.</a> |
| DEP_E176 | cloud bridge encountered duplicate alias on the cloud component. | change the alias name during deployment and ensure that there is no duplication of  alias name under the same routed network. For more information about alias, [click here.](/developer-guide/manage-software-cycle/communication-topologies/ros-support/#ros-environment-aliases-runtime-identity-assignment)</a> |
| DEP_E177 | cloud bridge on the cloud component encountered multiple ROS service origins. | re-onboard the device.</a> |
| DEP_E2xx | internal rapyuta.io error in the components deployed on cloud | report the issue together with the relevant details to the <a href="#" onclick="javascript:FreshWidget.show();">support team</a> |
| DEP_E3xx | internal rapyuta.io error in the components deployed on a device | report the issue together with the relevant details to the <a href="#" onclick="javascript:FreshWidget.show();">support team</a> |
| DEP_E4xx | internal rapyuta.io error | report the issue together with the relevant details to the <a href="#" onclick="javascript:FreshWidget.show();">support team</a> |


## Restart Policy
Unlike deployments running on the cloud, which automatically restart
if stopped due to an error, deployments that are running on devices
do not automatically restart if exited due to an error or when devices
are rebooted.

You can configure the behavior of deployments running on devices by
setting their restart policies. There are three kinds of restart policies
available for a device deployment:

* **Always**    
  Always restart deployments if the deployment executables are in an
  error state or if the device is rebooted.
* **On-failure**    
  Restart deployments only if the deployment executables exit due to an
  error, and the exit code is non-zero.
* **Never**    
  Do not restart deployments under any circumstance.

There are a couple of *exceptions* while applying the restart policies:

* Restarting a deployment running on a device may fail if its executables
  are missing in **$PATH**. It is due to the components of the
  deployment failing to start.
* rapyuta.io components that are shared between deployments running on the
  same device (like ROS Master) will have the same restart policy as
  that of the first deployment on a device irrespective of the restart
  policies of any subsequent deployments on the device.
* If a deployment running on a device is stopped
  manually by stopping its docker container,
  both **on-failure** and **always** policies (if selected) are not
  applied unless the device is rebooted.

{{% notice info %}}
You can modify or override the initial setting of restart policy while
deploying a package. Read [deploying a package](/developer-guide/manage-software-cycle/deployments/#deploying-a-package) topic to learn how to do so.
{{% /notice %}}

For a deployment running on a device, the variable
**Restart Count** (on the deployment details page) represents the
number of times the deployment has restarted due to restarting of
deployment components.

## Network Configuration for Executables
 After you deploy a package that contains ROS components, you can view the cloud bridge and routed network statuses for each component used in the package. The rapyuta.io platform relies on a sub-component called the cloud bridge for implicitly establishing a communication channel between two or more ROS environments.
 {{% notice note %}}
cloud bridge instances are automatically generated for the ROS components only.
{{% /notice %}}	
 The rapyuta.io platform also displays the warning counts in form of a histogram graph for last 24 hours. To view the histogram, click the warning icon next to the cloud bridge status as displayed in the following image.

 ![Modify restart policy](/images/multi-robot-communication/cb-warning-log.png?classes=border,shadow&width=40pc)

The following table displays the field description of the network configuration details section.

| Field | Description |
| ------ | ----------- |
| Name/ID | Displays a unique name or ID of the cloud bridge component that is generated for a ROS component. |
| Network | Displays the associated routed network for the component. |
| Routing Status | Displays the following cloud bridge statuses of each component for a package. <ul><li>If the cloud bridge is running successfully, then the status becomes **running**.</li><li>If the cloud bridge is running successfully with warnings, then the status becomes **running** with a warning icon. You can click on the warning icon to view the histogram of the warning messages those occured in the last 24 hours. You can also view the historical logs by clicking the warning message count bar in the histogram graph. It takes you to the [hitorical log section](/developer-guide/tooling-automation/logging/deployment-logs/#stdout-logs). You can also view the [live logs](/developer-guide/tooling-automation/logging/deployment-logs/#indexed-logs) of the cloud bridge generated for the ROS components in a deployment.</li><li>If the cloud bridge is failed due to some error, then the status becomes **error**.</li></ul> |
|Network Status | Displays the following routed network statuses of each component for a package. <ul><li>If the routed network is running successfully, then the status becomes **running**.</li><li>If the routed network is failed due to some error, then the status becomes **error**.</li></ul> |

## Deploying A Package
To deploy a package in rapyuta.io, follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Select the package you want to deploy.
3. Click **Deploy package**.
4. In the **Name of deployment** box, enter a name for the specific deployment
you are creating for the package.
5. A *LABEL* is a key-value pair. If you want to add a label, click **Add label**.
6. If a component of the package has *cloud* runtime, skip to instruction 9.
7. If a component of the package has *device* runtime, you must select the device
you want to deploy the component on. Click **Refresh the list of online
devices** to retrieve an updated list of online devices.
8. Select the device from the **Select device for deploying the component** drop-down list.
{{% notice info %}}
The list of devices is a set of online devices, which are pre-filtered to match
the architecture (amd64, arm32v7, arm64v8) and device runtime (docker or preinstalled)
required by the component in question.
{{% /notice %}}
9. If the package has a component with `Is ROS` true, then you will need to select **Routed Network** from the drop-down list.
    * If there are no **Routed Network** successfully running, you would not be able to deploy the package. Please create a [Routed Network](/build-solutions/sample-walkthroughs/routed-network) first. 
    * If you have a cloud component in your package, you will be able to select only cloud routed networks.
10. If you want to add a dependent deployment, click **Add dependency**, and select
a deployment you want to add as a dependency from the drop-down list of
deployment IDs.
11. If you want to add a volume, click **Add volume**. Ensure that a running volume
deployment is available before you add one.
12. If you want to modify the initial setting of restart policy of components with ***device runtime***, click **Modify**.
![Modify restart policy](/images/dev-guide/deployments/modify-restart-policy.png?classes=border,shadow&width=40pc)
13.  Click **CREATE DEPLOYMENT** > **Confirm**.

![Deploy demo package](/images/dev-guide/manage-software-lifecycle/deployment-routed-network.png?classes=border,shadow&width=40pc)
You will be redirected to the **Details** page of the newly created deployment.
The package is successfully deployed when the green colored bar moves from
**In progress** to **Provisioning** to **Succeeded** indicating that the
**DEPLOYMENT PHASE** has *succeeded* and the deployment **STATUS** is **Running**.

![Deployment example](/images/getting-started/deploy-pkg/demo-deployment.png?classes=border,shadow&width=50pc)

Furthermore, if dependent deployments are added then each dependency's **STATUS**
must read **Running**.

You may analyze the corresponding
[deployment logs](/developer-guide/tooling-automation/logging/deployment-logs/)
generated while deploying a package.

If a deployment fails, the **DEPLOYMENT PHASE** will
read **Failed to start**. You may have to click **Deprovision Deployment**, delete the package, create the package all over again, and try deploying it.

## Update/Re-Deploy In-Place
This feature allows users to re-deploy a running  a deployment without stopping and while retaining its ID, dependencies, configuration and endpoints. 
During the development phase this enables developers to switch between [newer or older build](/developer-guide/create-software-packages/builds/trigger-rollback/) 
version in a package without having to recreate a new package resource.
It is also useful in scenarios when a developer fixes and pushes a new image of software to docker repository with an identical tag 
and wants to pull in the version with all the changes and fixes into the running deployment. 

This is particularly useful in the case of a dependent deployment, as you do not need to deprovision all the deployments when a single deployment needs an update thus saving time.

{{% notice info %}}
The "in-place" Update/Redeploy feature is currently supported only on containers leveraging a [containerized device runtime](/developer-guide/manage-machines/device-runtime/#containerized-docker-runtime) 
and in the cloud. This feature is unavailable for Device components powered by the [pre-installed](/developer-guide/manage-machines/device-runtime/#preinstalled). 
{{% /notice %}}

To update/re-deploy a deployment, follow the steps:

1. On the left navigation bar, click **DEPLOYMENTS**.
2. Select the deployment that you want to update, and click Update Deployment.
The **Update Deployment** page appears.
3. The **Update Deployment** page lists all the components added to the package. Click the **Update** field next to the component that you want to update.
You can select at least one or more than one component to update.
4. Click **Update**.

It takes a few minutes and the deployment is updated. You can view the details of updated deployment in the **Details** tab.


**Update Deployment** can be done when [DEPLOYMENT PHASE](#phases) is either **Succeeded** or **Failed To Update**, 
on any other Deployment Phase the **Update Deployment** button will be disabled. 
In case of **Failed To Update**, you can check the **Historical Logs** but the **Live Logs** and **Shell Access** tabs will be disabled. 

{{% notice info %}}
In case your deployment goes to **Failed To Update**, it will show appropriate error code like 
[DEP_E151] (/developer-guide/manage-software-cycle/deployments/#error-codes) 
which means **device is either offline or not reachable**.
If you are not sure about the Error, please <a href="#" onclick="javascript:FreshWidget.show();">contact support</a>.
{{% /notice %}}	


You can see the **Deployment Generation** in the **Details** tab of the deployment. The generation increments by 1 for each update deployment. 
Suppose the current deployment generation is _i_ and if the user does Update Deployment then the new deployment generation will be _(i+1)_.


![Update deployment](/images/dev-guide/deployments/update-deployment.png?classes=border,shadow&width=55pc)



![Update deployment component](/images/dev-guide/deployments/update-deployment-component.png?classes=border,shadow&width=35pc)


You can click the **History** tab to view the update deployment history. It shows information like Time, Generation, 
User who updated the deployment and Deployment Status. For successful update, it shows a _green success icon_ in **Deployment Status**. 
While in case of update deployment failure (due to network issue or device being offline), it shows a _red failure icon_ in **Deployment Status**.  



![Update deployment history](/images/dev-guide/deployments/update-deployment-history.png?classes=border,shadow&width=60pc)

{{% notice info %}}
When **Update Deployment** is triggered, all the _replicas_ are deleted gracefully and the rapyuta.io platform automatically re-creates new replicas for the _component_.
{{% /notice %}}	
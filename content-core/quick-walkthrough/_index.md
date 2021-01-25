---
title: "Quick Walkthrough"
description:
type: quick-walkthrough
date: 2019-10-24T16:39:42+05:30
pre: "<b>3. </b>"
weight: 100
---

This tutorial will show you how to set up and run a variation of the ROS
Turtlesim on rapyuta.io. This tutorial targets individuals who like to get their hands dirty and explore a wider variety of functionality found in the rapyuta.io platform. 

In order to attain a deeper understanding of rapyuta.io it is highly recommended that the developer refer to the [developer guide](/developer-guide) section for in-depth documentation on each topic and/or pick from one of the more nuanced [sample walkthroughs](/build-solutions/sample-walkthroughs/) that deal with individual concepts and topics.

Some of the packages used in this tutorial
can be reused to control real robots such as [Turtlebot](https://www.turtlebot.com/).

## Learning objectives
In this tutorial, you will learn how to add and deploy packages using
[rapyuta.io console](https://console.rapyuta.io). Specifically, you'll
learn how to:

1. Create [Build](/developer-guide/create-software-packages/builds/), converting a public git repository to a container image. 
2. Source your packages from a public docker registry and builds.
3. Expose ROS topics, services, actions, and network endpoints as component-level interfaces.
4. Create [Routed Networks](/build-solutions/sample-walkthroughs/routed-network) and dependent deployments.
5. Control a Turtlebot in simulation through a web browser.

## Prerequisites

1. You should have access to a computer with an internet connection.
   Ensure that the latest [Google Chrome](https://www.google.com/chrome/) browser is installed on it.
2. You should be familiar with the following tools:
	* [Git](https://git-scm.com/doc)
	* [Docker](https://docs.docker.com/get-started/)
	* Robot Operating System ([ROS](http://wiki.ros.org/kinetic)) and its
	[topics](http://wiki.ros.org/Topics), [nodes](http://wiki.ros.org/Nodes)
	and [services](http://wiki.ros.org/Services).

## Difficulty
Advanced

## Estimated time
40 minutes

## Software architecture
Turtlesim’s software consists of packages or modules. Each package has a
specific purpose and contributes to the whole of Turtlesim. A modularized
software is easy to maintain, share, and scale. A package contains interfaces
that determine how it may interact with other packages. A deployment of a
package may depend on deployments of other packages. Learn more about
Turtlesim's [software architecture](./software-architecture) and [constituent packages](./packages)


## Creating the build
To create the build, follow below steps. Skip the following steps if you have already created an *io-tutorials* build earlier.

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**
3. In the Build Name box, enter a name for the build say `io-tutorials` 
4. In the **Git repository** box, enter the url address : 
`https://github.com/rapyuta/io_tutorials` and select **Build Recipe** as Catkin.
5. Go to the next step and click on next, the build will be created.

The build takes about two to five minutes to build the source code in the *io_tutorials*
repository into a running docker container. You may analyse the corresponding
[build logs](/developer-guide/tooling-automation/logging/build-logs/), which help debug failing builds.

Please proceed to creation of package once the build is Complete.



## Create the Turtle package
The Turtle package emulates the behavior of a Turtlebot through the Simulator.
There may be multiple instances (deployments) of Turtle component, each
representing a single robot entity. In a real-world implementation,
the Turtle package would directly control a robot. Since this is a simulation,
the dynamics of the Turtle's behavior is obtained from the Simulator. Learn
more about [Turtle package](./packages/#turtle-package).

To add _Turtle_ package using [rapyuta.io console](https://console.rapyuta.io),
follow the steps:

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You should provide information about the package such as the name of the
   package, its version, whether it is a singleton package and a short
   description.
   1. In the **Package Name** box, enter a name say `Turtle`
   2. In the **Package Version** box, specify a version for the package. Initially, the version is set to the default value _1.0.0_
   3. Make sure **Is singleton package** is not selected.
   4. Ensure **Is bindable package** is selected.
   5. In the **Description** box, provide a summary of the package. For
      example, `An emulation of Turtlebot`
   6. Click **NEXT**.
3. In the **Component Name** box, enter a name for the component such as `turtle`      
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen -, and an underscore _ character, and must not start with a digit.
{{% /notice %}}
4. Select **Cloud** as the **Component Runtime**.
5. Ensure **Is ROS Component** is selected.
6. Set **Replicas to run the component** to number 1 (default value).
7. In the **Executable Name** box, type in a name for the executable. For example,
   `turtle_executable`       
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9], hyphen -, and an underscore _ character, and must not start with a digit.
{{% /notice %}}
8. For **Executable Type**, click on **Builds**.
9. In the **Choose Build** select the Build (`io-tutorials`) [created above](/quick-walkthrough/#creating-the-build)
	from the drop-down list.	
10. In the **Command to run in the docker container** box, enter the command:
    ```bash
	roslaunch io_turtle turtle.launch
    ```

	Ensure you always execute the *roslaunch* command for explicitly starting the
	[ROS Master](http://wiki.ros.org/Master). However, it's not recommended to
	run the *rosrun* command because the ROS Master will fail to start, and thus,
	the deployment fails.
11. Add `/pose` ROS topic with low QoS because there is no need to guarantee
    the delivery of each control message. Instead, it is essential to receive
	the most recent message at the expense of losing some information.
	Click **Add ROS topic** > enter `/pose` in the **Name** box > click **Low** for **QoS**.
12. Add `/sim/cmd_vel` ROS topic with high QoS because you will expect the
    Turtle to react when given such a command.
	Click **Add ROS topic** > enter `/sim/cmd_vel` in the **Name** box > select **High** for **QoS**.
13. To add a ROS action, click **Add ROS action**. In the **Name** box,
    enter `/turtle_0/goto_action`. Similarly, add another ROS action `/turtle_1/goto_action`
14. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

You can deploy multiple instances of the _Turtle_ package. To deploy up to two
instances of the package, you define two ROS actions. However, you may define
as many ROS actions as you would want to, and also deploy them on many Turtles.
The ROS actions are dynamically created when the _Turtle_ registers with the
_Command Center_. Each _Turtle_ uses one of the defined actions.

## Create User Interface package
The _User Interface_ is an interactive, web-based visualization of the
original ROS Turtlesim. You can visualize the state of the Turtles, pass control commands to the Turtles. A docker container encapsulates its application code and dependencies. It communicates with the
_Command Center_ through a WebSecureSocket. It is the only non-ROS
package based on Nginix. Learn more about
[User Interface package](./packages/#user-interface-package).

To add _User Interface_ package using the [console](https://console.rapyuta.io), follow the instructions:

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You should provide information about the package, such as the name of the package,
   its version, whether it is a singleton package, and a short description of the package.
   1. In the **Package Name** box, enter a name say `User Interface`
   2. In the **Package Version** box, specify a version for the package. Initially,
      the version is set to the default value _1.0.0_
   3. Make sure **Is singleton package** is not selected.
   4. Ensure **Is bindable package** is selected.
   5. In the **Description** box, enter a summary of the package such as
      `An interactive, web based version of ROS Turtlesim's user interface`
   6. Click **NEXT**.
3. In the **Component Name** box, enter a name for the component say `user_interface`   
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen -, and an underscore _ character, and must not start with a digit.
{{% /notice %}}
4. Select **Cloud** for **Component Runtime**.
5. Deselect **Is ROS Component** because _User Interface_ is a non-ROS package.
6. Set **Replicas to run the component** to number 1 (default value).
7. In the **Executable Name** box, enter a name for the executable say `uiexecutable`    
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9], hyphen -, and an underscore _ character, and must not start with a digit.
{{% /notice %}}
8. Select **Docker** for **Executable Type**.
9. In the **Docker image** box, enter the address of the docker image that contains
   the application code. The address of the image is `rrdockerhub/io_turtlesim_ui`
   as the image is held in the public docker registry, [DockerHub](https://hub.docker.com/).
10. The **Command to run in the docker container** remains empty because the docker image will automatically execute when you deploy the _User Interface_ package.
11. Deselect **Run command from bash shell** checkbox.
12. Expose a communication endpoint to access _User Interface_ as well as interact
    with the Turtles. To add an endpoint, follow the steps:
	1. Click **Add endpoint**.
	2. In the **Endpoint Name** box, enter a name say `UserInterface`
	3. Select **Exposed externally**.
	4. Click **HTTPS/WSS** for **Protocol**.
	5. The **Port** is set to the default value 443 for the HTTPS/WSS protocol.
	6. In the **Target Port** box, enter the value `8080` (default port for Nginix).
13. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

## Create Simulator package
The _Simulator_ package replicates the physical dynamics of 2D robots and
their sensors. Learn more about [Simulator package](./packages/#simulator-package).

To add the _Simulator_ package, follow the steps:

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You must provide information about the package, such as the name of the package,
   its version, whether it is a singleton package, and a short description.
   1. In the **Package Name** box, type in the name for the package say
      `Simulator`
   2. In the **Package Version** box, enter the version of the package.
      The default value is set to _1.0.0_
   3. Ensure **Is singleton package** is not selected.
   4. Ensure **Is bindable package** is selected.
   5. In the **Description** box, enter a short summary of the package such as
      `Simulator simulates physical dynamics of 2D robots and their sensors`
   6. Click **NEXT**.
3. In the **Component Name** box, type in a name for the component, say `simulator`  
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9], hyphen -, and an underscore _ character, and must not start with a digit.
{{% /notice %}}
4. Select **Cloud** for **Component Runtime**.
5. Ensure **Is ROS Component** is selected.
6. Set the value of **Replicas to run the component** to number 1 (default value).
7. In the **Executable Name** box, enter a name for the executable say
   `simulator_executable`   
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9], hyphen -, and an underscore _ character, and must not start with a digit.
{{% /notice %}}
8. For **Executable Type**, click on **Builds**.
9. In the **Choose Build** select the Build (`io-tutorials`) [created above](/quick-walkthrough/#creating-the-build)
	from the drop-down list.
10. In the **Command to run in the docker container** box, enter the command:
    ```bash
    roslaunch io_turtle_sim_env simulation.launch
    ```

    Ensure you always execute the *roslaunch* command for explicitly starting the
    [ROS Master](http://wiki.ros.org/Master). However, it's not recommended to
    run the *rosrun* command because the ROS Master will fail to start, and
    thus the deployment fails.
11. To add `/sim/pose` ROS topic, click **Add ROS topic**. In the **Name** box, type in
    `/sim/pose` and select **Low** for **QoS**.
12. To add another ROS topic, `/sim/sensors`, click **Add ROS topic** again.
    In the **Name** box, type in `/sim/sensors` and select **Low** for **QoS**.    
{{% notice info %}}
The value of **QoS** is set to **Low** because it is not necessary to guarantee
the delivery of each topic. Instead, it is essential to receive the most
recent message at the expense of losing some information.
{{% /notice %}}
13. To add a ROS service, click **Add ROS service**. In the **Name** box, enter the name of the service `/register_sim_turtle`. Similarly, add another
ROS service `/teleport_turtle`.
14. Click **NEXT**.
15. Click **CONFIRM PACKAGE CREATION**.

## Create Command Center package
The _Command Center_ is a node for registering _Turtles_, and a router for
passing control commands and telemetry messages between _User Interface_
and _Turtles_. Learn more about [Command Center package](./packages/#command-center-package).

To add _Command Center_ package, follow the steps:

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. You must provide information about the package such as the name of the
   package, its version, whether it is a singleton package, and a
   short description.
   1. In the **Package Name** box, type in the name for the package
      say `Command Center`
   2. In the **Package Version** box, enter the version of the package.
      The default value is _1.0.0_
   3. Ensure **Is singleton package** is not selected.
   4. Ensure **Is bindable package** is selected.
   5. In the **Description** box, enter a short description of the package such
      as `Command Center is responsible for communication between
      User Interface and Turtle`
   6. Click **NEXT**.
3. In the **Component Name** box, enter a name for the component
   say `command_center`   
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits
[0-9], hyphen - and an underscore _ character, and must not start with a digit.
{{% /notice %}}
4. Select **Cloud** for the **Component Runtime**.
5. Ensure **Is ROS Component** is selected.
6. Set the value of **Replicas to run the component** to number 1 (default value).
7. In the **Executable Name** box, type in a name for the executable
   say `ccexecutable`
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z],
digits [0-9], hyphen - and an underscore _ character, and must not start with a digit.
{{% /notice %}}
8. For **Executable Type**, click on **Builds**.
9. In the **Choose Build** select the Build (`io-tutorials`) [created above](/quick-walkthrough/#creating-the-build)
	from the drop-down list.
	
10. In the **Command to run in the docker container** box, enter the command:
    ```bash
    roslaunch io_turtle_command_center command_center.launch
    ```

    Ensure you always execute the *roslaunch* command for explicitly starting
    the ROS Master. However, it's not recommended to run the
    *rosrun* command because the [ROS Master](http://wiki.ros.org/Master) will fail to start, and thus the deployment fails.

11. You must expose a communication network endpoint for
    publicly accessing _Command Center_.
    1. Click **Add endpoint**.
    2. In the **Endpoint Name** box, enter `WS_ROSBRIDGE`
    3. Make **Exposed externally** checkbox is selected.
    4. Click **HTTPS/WSS** under **Protocol**.
    5. The value of **Port** is automatically set to _443_ because the Protocol is HTTPS/WSS.
    6. In the **Target Port** box, type in `9090`

12. To add a ROS topic, click **Add ROS topic** > enter `/cmd_vel` in the **Name** box > click **Low** for **QoS**.

13. To add a ROS service, click **Add ROS service** > enter `/register_turtle` in the **Name** box.

14. The environment variables: *WS_ADDR*, *WS_PORT*  defined in the ROS launch file, determine the WebSecureSocket address, port respectively. You can adjust
    their values as configuration parameters defined in the [console](https://console.rapyuta.io)
    during the process of deploying the *command_center* component.
    To add *WS_ADDR* as configuration parameter, follow the steps:
    1. Under **CONFIGURATION PARAMETERS**, click **Add Parameter**.
    2. In the **Name** box, enter `WS_ADDR`
    3. In the **Default** value box, type in `0.0.0.0`
    4. In the **Description** box, type in `WebSocket Address`
    5. Ensure **This parameter is exposed externally** is not selected.    
    Similarly, add *WS_PORT* configuration parameter. Set **Name** as `WS_PORT` and
    **Default** value to `9090`, and describe it as `WebSocket Port`.
    
15. Click **NEXT** to move to **Additional Information** page.

16. Click **CONFIRM PACKAGE CREATION**.



## Deploy packages
In this tutorial, you have created four packages. They are:

1. Turtle
2. User Interface
3. Simulator
4. Command Center

You may click **CATALOG** to view all of the above packages in one place.

Out of these packages Turtle, Simulator and Command Center packages would share a routed network. Routed network enables ROS communication between different ROS package deployments.
Binding a routed network to your deployment will enable other deployments on the same routed network to consume ROS topics/services/actions as defined in the package.

You will need to create a routed network for them. Please follow this [tutorial](/build-solutions/sample-walkthroughs/routed-network#creating-cloud-routed-network) on how to create a routed network.

## Deploy Simulator package
To deploy the _Simulator_ package, follow the steps:

1. Click **CATALOG** > select _Simulator_ package > click **Deploy package**.

2. In the **Name of the deployment** box, type in a name for the deployment
   you are creating say `SIMULATOR deployment`
   
3. Click on **ROUTED NETWORK** > **Add**, select the routed network created by you from the dropdown list.    

4. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the deployment **Details** page where a green progress bar moves up to **Succeeded** along with **Status:Running** point indicating that the
**DEPLOYMENT PHASE** has **Succeeded**, and the deployment **STATUS** is **Running**.

## Deploy Command Center package
To deploy the _Command Center_ package, follow the steps:

1. Click **CATALOG**  >  select **COMMAND CENTER** package  >  click **Deploy package**.
2. In the **Name of deployment** box, enter a name for the deployment say
   `COMMAND CENTER deployment`
3. Ensure that the value of **WS_ADDR** is **0.0.0.0** and **WS_PORT** is **9090**
4. Click on **ROUTED NETWORK** > **Add**, select the routed network created by you from the dropdown list.
5. Click **CREATE DEPLOYMENT**  >  **Confirm**.

You are redirected to the deployment's **Details** page where a green progress bar
moves up to **Succeeded** and **Status:Running** point indicating that the
**DEPLOYMENT PHASE** has **Succeeded**, and the deployment **STATUS** is **running**.
Since _COMMAND CENTER deployment_ depends on _SIMULATOR deployment_,
ensure that the dependent deployment's **STATUS** is **running** as well.

**If you're having trouble with getting the websocket connection to work with `rosbridge-server`, please check [this](/build-solutions/quirks/rosbridge-compatibility).**

## Deploy Turtle package
To deploy the _Turtle_ package, follow the steps:
 
1. Click **CATALOG** > select **Turtle** package > click **Deploy package**.
2. In the **Name of the deployment box**, type in a name for the deployment you are creating say  `TURTLE deployment` 
3. Click on **ROUTED NETWORK** > **Add**, select the routed network created by you from the dropdown list.
4. Click **CREATE DEPLOYMENT** > **Confirm**.

## Deploy User Interface package
When you deploy _User Interface_ package (non-ROS package), ensure you add the
_Command Center_ deployment as a dependent deployment. That is a deployment of
_User Interface_ is based on that of _Command Center_.

The procedure to deploy the _User Interface_ package is the same as deploying
the _Command Center_ package. However, provide a different name for the deployment
say `USER INTERFACE deployment` To add a dependent deployment, click **Add dependency**
and select the **COMMAND CENTER deployment** ID from the drop-down list.
Click **CREATE DEPLOYMENT**  >  **Confirm**.

If the **DEPLOYMENT PHASE** is **Succeeded** and the **STATUS** is running for
the dependent deployment as well, _User Interface_ package is successfully deployed.

The **NETWORK ENDPOINTS** generates a URL on the deployment **Details** tab.
Copy this specific URL (your URL address will be different from that shown in the screen
capture), paste it in the web browser, and run it. You will view the
Turtlesim's user interface without any Turtles. However, if you do not
see the Turtlesim user interface, try refreshing the web page for
a couple of times.

![Network Endpoint for User Interface package](/images/tutorials/turtlesim/UI-endpoint.png?classes=border,shadow&width=60pc)



If the deployment fails, click **Deprovision deployment**; delete the
corresponding package; create the package and deploy it again.

On the left navigation bar, click **DEPLOYMENTS** to view a list of
four deployments that you created.

An illustration of the resultant dependency graph is:

![Dependency graph of Turtlesim](/images/tutorials/turtlesim/dependency-graph.png?classes=border,shadow&width=60pc)

Refresh the Turtlesim User Interface web page to view a single Turtle
randomly placed in the environment.

![Turtlesim simulation](/images/tutorials/turtlesim/turtlesim-viewer.png?classes=border,shadow&width=40pc)

Try moving the Turtle around using the command interface. If the Turtle
behaves as you expect, you have successfully finished the tutorial.

You can deploy a second Turtle (in the same way as the first). You can now
command two turtles simultaneously.

## Cleaning up
A running deployment of a package consumes cloud resources. It is recommended
to stop a running deployment when you do not need it. To deprovision (stop) a
running deployment, follow the steps:

1. On the left navigation bar, click **DEPLOYMENTS**.
2. Click **Deprovision** against the running deployment ID you want to stop.

If the deployment is successfully deprovisioned, you will view a
**DEPLOYMENT STOPPED** message and the corresponding deployment ID's **PHASE**
reads **Deployment stopped**.

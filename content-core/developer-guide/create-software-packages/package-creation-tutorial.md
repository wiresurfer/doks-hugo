---
title: "Package Creation Tutorial"
description:
type: developer-guide
date: 2019-10-25T12:35:41+05:30
pre: "4. "
weight: 285
---

## Creating the build
Follow below steps to create the build : 

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**.
   The **Create new build** page is displayed.
3. In the Build Name box, enter a name for the build say `demoBuild` 
4. In the **Git repository** box, enter the url address : `https://github.com/rapyuta/io_tutorials` 
and select **Build Recipe** as Catkin.
5. Go to the next step and click on next, the build will be created. For  more information about the field descriptions of the **Create new build** page, [click here](/developer-guide/create-software-packages/builds/build-creation/#creating-build-by-catkin-recipe).


## Creating the package
To add a new package using rapyuta.io, follow the steps:

1. Click **CATALOG** > **ADD NEW PACKAGE**.
2. **Package information** contains the basic information of a package like the name of the package, its version, and a summary of the package.
   1. In the **Package Name** box, type in the name for the package. For instance, let's call the package as `Demo Package`.
   2. In the **Package Version** box, enter the version of the package. It is set to _1.0.0_ by default.
   3. Ensure that **Is a singleton package** is ***not selected***.
   4. Ensure that **Is bindable package** checkbox ***is selected***.
   5. It is optional to describe the package in the **Description** box.
3. Click **NEXT**.
4. **Components** contains details about the package's components and executables of each component. Fill in the **COMPONENT METADATA** information.
   1. In the **Component Name** box, enter the name of the component. For example, `Talker`.
{{% notice info %}}
The name of a component must consist of alphabets [A-Z, a-z], digits [0-9] and an underscore _ character, and must not begin with a digit.
{{% /notice %}}
   2. A component of a package can either have a device runtime or a cloud runtime. Select **Cloud** as **Component Runtime** in this example.
   3. Ensure that **Is ROS Component** is selected.
   4. Ensure that the value of the **Replicas to run the component** is set to the default value **1**
![Component metadata](/images/getting-started/create-new-pkg/component-metadata.png?classes=border,shadow&width=30pc)
1. Add one or more **EXECUTABLES** to the component.
   1. In the **Executable Name** box, type in the name of the executable, say `talkerExecutable`.
{{% notice info %}}
The name of an executable must consist of alphabets [A-Z, a-z], digits [0-9] and an underscore _ character, and must not begin with a digit.
{{% /notice %}}
   2. For **Executable Type**, click on **Builds**.
{{% notice note %}}
The maximum size of the docker image is 10 GB for cloud deployment
{{% /notice %}}
		
   3. In the **Choose Build** select the Build [created above](/developer-guide/create-software-packages/package-creation-tutorial/#creating-the-build)
   from the drop-down list.
   4. In the **Command to run in the docker container** box, enter the command:`roslaunch talker talker.launch`     
    Ensure that you always execute the *roslaunch* command to explicitly start the [ROS
	Master](http://wiki.ros.org/Master) instead of running the *rosrun* command,
	because the ROS Master will fail to start on *rosrun* command in the console,
	and eventually, the deployment will fail.
![Executable information](/images/getting-started/create-new-pkg/exec-details.png?classes=border,shadow&width=50pc)
1. Add a ROS topic by clicking **Add ROS topic**, specify the name of the ROS topic in the **Name** box, and choosing **Maximum QoS**. In this example, the ROS topic `/telemetry` is added.
![Add ROS topic](/images/getting-started/create-new-pkg/add-ros-topic.png?classes=border,shadow&width=50pc)
2. To add another component to the same package, click **+** as shown in the image and repeat instructions in steps 4 through 6.
![Add another component](/images/getting-started/create-new-pkg/add-another-component.png?classes=border,shadow&width=30pc) 
3. Click **NEXT** > **CONFIRM PACKAGE CREATION**.


![Demo package](/images/getting-started/create-new-pkg/demo-pkg.png?classes=border,shadow&width=50pc)

You can download the manifest of the package by clicking
**Download Manifest**. You may also clone the package if you want to
modify it by clicking **Clone Package**. You can trigger a new build by clicking **Trigger build** and deploy the package by
clicking **Deploy package**.

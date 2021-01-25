---
title: "Build Creation and Deletion"
description:
type: developer-guide
date: 2019-10-24T13:46:19+05:30
pre: "a. "
weight: 320
---

## Creating Build by Catkin recipe 
Follow the below steps to create the build by catkin recipe : 

1. On the left navigation bar, click **BUILDS**.
2. Click on **ADD NEW BUILD**.
3. Under **Git info** tab, do the following.</br>
	a. In the **Build Name** field, enter a name for the build.</br>
	b. Select the build recipe as **Catkin**.</br>
	c. In the **Repository (URL)** field, enter the URL of the git repository from which you want to create a build. For example, `https://github.com/rapyuta/io_tutorials`.</br>
	d. Optionally, if the git repository is a private git, then click the **Private Git** radio button and select the credential from the **Credential** drop-down menu. If you have not created any secret for the repository, [create a source secret](/developer-guide/create-software-packages/secrets/sourcecode-repository/#creating-source-secret).</br>
	e. Optionally, you can specify the **Context Directory** field by entering the path of the directory where you want to create the build.</br>
	f. Click **Next**. 
![goo](/images/core-concepts/builds/build-creation/catkin-recipe.png?classes=border,shadow&width=30pc)
4. Under the **Build Info** tab, do the following.</br>
	a. In the **Architecture** area, select the processor architecture for the build. The available options are:</br>
	<ul>
	<li>arm32v7</li>
	<li>arm64v8</li>
	<li>amd64</li>
	</ul>
	b. Optionally, if the build has a simulation option, click the **Has Simulation** radio-button.</br>
	c. In the **ROS Version** area, select the ROS version. The available options are:</br>
	<ul>
	<li>Kinetic</li>
	<li>Melodic</li>
	</ul>
	d. Optionally, to add ROS parameter to the build, in the **CATKIN BUILD PARAMETERS** area, click **Add Parameter**. For more information about the Catkin parameters, [click here.](/developer-guide/create-software-packages/builds/ros-support/)</br>
	e. Click **Next**.
![goo](/images/core-concepts/builds/build-creation/catkin-build-info.png?classes=border,shadow&width=30pc)
5. The *rapyuta.io* platform allows you to push and save the build image to an external docker registry.To push and save the build image to the private registry, do the following.</br>
	a. Under the **Docker Secret** tab, click the **Docker Push Secret**  toggle button.</br>
	b. In the **Image Repository** field, enter the URL of the private repository where you want to push and save the image for later usage. For example, *docker.io/your-username/repo-name*. </br>
	c. From the **Push secret** drop-down menu, select the secret for the repository. If you have not created any secret for the repository, [create a secret](/developer-guide/create-software-packages/secrets/docker-registry/#creating-a-docker-secret).</br>
	d. Click **Next**.
![goo](/images/core-concepts/builds/build-creation/catkin-push-secret.png?classes=border,shadow&width=30pc)
The build is created. To view the details of the build, [click here](/developer-guide/create-software-packages/builds/build-creation/#viewing-build-details).


## Creating Build by Docker recipe 
Follow the below steps to create the build by docker recipe : 

1. On the left navigation bar, click **BUILDS**
2. Click on **ADD NEW BUILD**.
3. Under **Git info** tab, do the following.</br>
	a. In the **Build Name** field, enter a name for the build.</br>
	b. Select the build recipe as **Docker**.</br>
	c. In the **Repository (URL)** field, enter the URL of the git repository from which you want to create a build. For example, `https://github.com/rapyuta/io_tutorials`.</br>
	d. Optionally, if the git repository is a private git, then click the **Private Git** toggle button and select the credential from the **Credential** drop-down menu. If you have not created any secret for the repository, [create a source secret](/developer-guide/create-software-packages/secrets/sourcecode-repository/#creating-source-secret).</br>
	e. Optionally, you can specify the **Context Directory** field by entering the path of the directory where you want to create the build.</br>
	f. Click **Next**. 
![goo](/images/core-concepts/builds/build-creation/docker-recipe.png?classes=border,shadow&width=30pc)
4. Under the **Build Info** tab, do the following.</br>
	a. In the **Architecture** area, select the processor architecture for the build. The available options are:</br>
	<ul>
	<li>arm32v7</li>
	<li>arm64v8</li>
	<li>amd64</li>
	</ul>
	b. In the **Dockerfile path** field, type the path of the docker file that contains the source code.</br>
	c. Optionally, if the build has ROS component, then click the **Has ROS Components** radio-button, select the ROS version as either **Kinetic** or **Melodic**, and if the build has a simulation option, click the **Has Simulation** radio-button.</br>
	d. Click **Next**.
![goo](/images/core-concepts/builds/build-creation/docker-build-info.png?classes=border,shadow&width=30pc)
5. The *rapyuta.io* platform allows you to push and save the build image to an external docker registry or to pull an image from a private repository. To push or pull a docker image file, do the following.</br>
	a. Under the **Docker Secret** tab, click the **Docker Push Secret**  toggle button.</br>
	b. In the **Image Repository** field, enter the URL of the private repository where you want to push and save the image for later usage. For example, *docker.io/your-username/repo-name*.</br>
	c. From the **Push secret** drop-down menu, select the secret for the repository. If you have not created any secret for the repository, [create a secret](/developer-guide/create-software-packages/secrets/docker-registry/#creating-a-docker-secret).</br>
	d. Optionally, to pull a secured docker image to the docker file, click the **Docker Pull secret** toggle button and select the secret for the repository. If you have not created any secret for the repository, [create a secret](/developer-guide/create-software-packages/secrets/docker-registry/#creating-a-docker-secret).</br> 
{{% notice note %}}
**Docker Pull Secret** field is only available if you are creating the build by docker recipe.</br>
{{% /notice %}}
	e. Click **Next**.
![goo](/images/core-concepts/builds/build-creation/docker-push-secret.png?classes=border,shadow&width=30pc)
The build is created. To view the details of the build, [click here](/developer-guide/create-software-packages/builds/build-creation/#viewing-build-details).

You can see the progress of the build, by clicking on the Build created in the **Builds** page. 
Click on **SHOW MORE** to get more details about the build, it will take you to the **Details** tab of the build.

## Viewing Build Details
After you have created a build by either Catkin or Docker recipe, you can view the details of the available builds in your project. Perform the following steps to view the details of a build.

1. On the left navigation bar, click **BUILDS**. All the available builds of the projects are displayed.
![goo](/images/core-concepts/builds/build-creation/builds.png?classes=border,shadow&width=45pc)
 The following table displays the details of the available builds.</br>


    |Field|Description|
    |-----|-----------|
    |**Name/ID**| Provides the name of the build.
    |**Status**| Provides the following status of the build. <ul><li>BuildInProgress: Displays when the build creation process is in progress.</li><li>Complete: Displays when the build creation process has been successfully completed.</li><li>BuildFailed: Displays if the build creation process has failed.</li></ul>
    |**Started**| Provides the time duration when the build creation process has started.
    |**Repository**| Provides the repository from which the build has been created.
    |**Action**| Allows you to [delete](/developer-guide/create-software-packages/builds/build-creation/#deleting-the-build), [trigger](/developer-guide/create-software-packages/builds/trigger-rollback/), or [clone](/developer-guide/create-software-packages/builds/build-cloning/)  a build. 

2. To view the details of a particular build, click the build. The following image is displayed.
![goo](/images/core-concepts/builds/build-creation/build-detail-action.png?classes=border,shadow&width=25pc)

The **Build Details** page allows you to do the following.</br>

* Click **[Delete](#deleting-the-build)** to delete the build. 
* Click **[Trigger](/developer-guide/create-software-packages/builds/trigger-rollback/)** to trigger a build. 
* Click **[Clone](/developer-guide/create-software-packages/builds/build-cloning/)** to clone a build within the same or to different projects.
* Click **Show More** to view the details of the build and view the [build log](/developer-guide/tooling-automation/logging/build-logs).

After clicking **Show More**,  the following image is displayed.
![goo](/images/core-concepts/builds/build-creation/build-details.png?classes=border,shadow&width=45pc)



## Deleting the build
Follow the below steps to delete the build :

1. On the left navigation bar, click **BUILDS**. It  displays all the builds available for a project.
2. Select the build that you want to delete. 
3. Click **Delete**.
4. Confirm on the build deletion message.


{{% notice note %}}
Please note that if the build **Status** is _BuildInProgress_, then user will not be able to **Delete** the build. Deletion of _BuildInProgress_ **Status** build 
will be failed with the error message : **can't delete the build since its corresponding build request is still in progress**.
{{% /notice %}}




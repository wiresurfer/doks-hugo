---
title: "Onboarding"
description:
type: developer-guide
date: 2019-10-24T11:47:21+05:30
pre: "3. "
weight: 240
---
To register a new device on rapyuta.io, follow the below steps:

1. On the left navigation bar, click **DEVICES**.
2. Click **ADD NEW DEVICE**.
3. You will provide a name for the device in the **Device Name** box.
4. You can choose to build the application's source code inside the device or outside it.
   1. Do ***not*** select **Use docker compose as default runtime** option if the application source code is already built locally in the device.
   2. Select **Use docker compose as default runtime** option if you want to build the application source code in rapyuta.io and push the build to the device remotely. On selecting this option, you will be asked to choose the version of ROS installed on the device.
   ![Select ROS version](/images/getting-started/add-new-device/select-ROS-version.png?classes=border,shadow&width=40pc)
5. You will provide the absolute path of your catkin workspace as
   the **ROS Catkin Workspace** value. If you are using custom rapyuta.io
   image on the device, your catkin workspace will be
   `/home/rapyuta/catkin_ws` where *catkin_ws* is the name of the catkin workspace.
   Otherwise, you will have a different absolute path for your catkin workspace.
   {{% notice info %}}
   The **ROS Catkin Workspace** can be empty, and you may provide this value on
   the device's **Details** page.
   {{% /notice %}}
6. In the **Description** box, enter a brief summary of the device.
7. Click **CONTINUE**.
    ![Device Details](/images/getting-started/add-new-device/device-details.png?classes=border,shadow&width=40pc)
8.  To copy the generated token (a unique device setup link), click **COPY**.    
   You must copy the token before it expires in *ten* minutes. To generate
   the token again, click **Token** on the **Devices** page.
   ![Device Token](/images/getting-started/add-new-device/device-token.png?classes=border,shadow&width=40pc)
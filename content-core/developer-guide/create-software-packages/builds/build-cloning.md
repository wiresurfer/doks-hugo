---
title: "Cloning a Build"
description:
type: developer-guide
date: 2020-12-25T12:39:12+05:30
pre: "b. "
weight: 325
---

The rapyuta.io platform allows you to clone a build within the same or different project and reduces the time and effort required to create a build from scratch.

Perform the following procedure to clone a build.

1. On the left navigation bar, click **BUILDS**. It displays all the builds available within a project.
2. Select the build that you want to clone and click **Clone Build**. 
3. A dialog box appears and prompts you to select the project where you want to clone the build. Select the project from the drop-down and click **Clone Build**.
![build-clone](/images/core-concepts/builds/build-clone.png?classes=border,shadow&width=25pc)

{{% notice info %}}
If you want to clone a project within the same project, you must rename the build name.
{{% /notice %}}


4. If the build that you are cloning has a secret, the rapyuta.io platform also allows you to clone the secret to the destined project or select a different secret that you want to use in the cloned build. From the **Destination** drop-down menu, select the secret that you want to use in the cloned build and click **Clone**.
![secret-clone](/images/core-concepts/builds/build-creation/secret-clone.png?classes=border,shadow&width=35pc)

{{% notice note %}}
The dialog box to clone secret appears only for the builds that have a secret.
{{% /notice %}}
5. The build creation page appears and allows you to modify the build details. For more information about build creation, [click here](/developer-guide/create-software-packages/builds/build-creation).</br>
6. After reviewing the field details of the build creation pages, click **Next**. The build is cloned to the selected project.
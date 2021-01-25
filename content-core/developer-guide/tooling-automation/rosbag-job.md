---
title: "Ros bag Jobs"
description:
type: developer-guide
date: 2020-10-28T13:22:44+05:30
pre: "2. "
weight: 515
---
A ROS bag is a file format in ROS for storing ROS message data. The rapyuta.io platform allows you to record the ROS messages (ROS topics) for ROS enabled components deployed on the cloud.

After you deploy a package, you can view the ROS bag jobs in the **ROS Bag Jobs** tab on the deployment details page.


If you have added a ROS bag job for a component either during package creation or deployment, You can add new ROS bag jobs for the same components in the **ROS Bag Jobs** tab after you have deployed the package. To add new jobs, click **Add new ROS Bag job**. To know more about the field description, [click here](/developer-guide/create-software-packages/ros-support/#creating-rosbag-jobs).

{{% notice note %}}
If you have not added any ROS bag job for a cloud component during deployment or package creation, you cannot add a ROS bag job later in the **ROS Bag Jobs** tab.

{{% /notice %}}

{{% notice info %}}
If the deployed component is a non-ROS component, or you have not added any ROS bag job during package creation or deployment, or the selected runtime is device, the **ROS Bag Jobs** tab is not available.
{{% /notice %}}

The **ROS Bag Jobs** tab allows you to do the following.


* [Create a ROS bag job](/developer-guide/create-software-packages/ros-support/#creating-ros-bag-jobs) 
* [View ROS bag job details](/developer-guide/tooling-automation/rosbag-job/#viewing-ros-bag-job-details)
* [View ROS bag file details](/developer-guide/tooling-automation/rosbag-job/#viewing-ros-bag-file-details)

### Viewing ROS Bag Job Details

The rapyuta.io platform lists all the available ROS bag jobs under each component. To view the details of a ROS bag job, click the ROS bag job and then click the **Job Details** tab. You can view the ROS bag details, for example, the topics that are being recorded by the job or the message compression detail as displayed in the following image.
![rosbag-job-details](/images/dev-guide/rosbag-jobs/rosbag-job-details.png?classes=border,shadow&width=30pc)

### Viewing ROS Bag File Details

 A ROS bag file is uploaded to the platform automatically if the ROS bag job is stopped or if the deployment is de-provisioned. To stop the ROS bag job, click **Stop** next to the ROS bag job name. A pop-up warning message appears. Click **Yes**. The platform takes some time and the details of the recorded topics are available. You can view the following details.
The **ROS Bag Jobs** tab lists all the running ROS bag jobs for the deployment. 
![rosbag-jobs](/images/dev-guide/rosbag-jobs/rosbag-jobs.png?classes=border,shadow&width=55pc)

* **Bag Name**: Displays the name of the ROS bag file.
* **Status**: Displays whether the recorded ROS bag file is uploading, uploaded, or in the error state. If the status is **Uploading**, it implies that the ROS bag file is getting uploaded to the platform after the ROS bag job is stopped or the deployment is de-provisioned. If the status is **Uploaded**, it implies that the file is uploaded to the platform successfully and you can download the file to your local system for further analysis and troubleshooting. If the status is in **Error** state, it implies that the recording of the ROS bag file is not successful and the ROS bag file is not available. If the ROS bag job goes into error state, click the **Error** link to raise a support ticket for a resolution.

* **Messages**: Displays the number of recorded messages.
* **Size**: Displays the file size.
* **Start**: Displays the start time of the recording.
* **End**: Displays when the recording is stopped.
* **Duration**: Displays the total time duration for the ROS bag job that was run.
* **Indexed**: If the **Indexed** field is marked with a green tick icon, it implies that the ROS bag file is indexed and contains valid recordings of topics. If the **Indexed** field is marked with a red cross icon, it implies that the ROS bag file is not indexed and does not contain valid recordings of topics.
* **Actions**: You can either download the ROS bag file or delete the file.

#### Available Actions on a ROS Bag File

After a running ROS bag job is uploaded to the platform, you can perform the following available actions.

* To have a quick view of the details of the recorded topics, click the ROS bag file that is uploaded. A pop-up message box appears detailing the recorded topics. 
![bag-topics](/images/dev-guide/rosbag-jobs/topic-bags.png?classes=border,shadow&width=30pc)
* To download the available ROS bag file, click the download icon.
* To delete the uploaded ROS bag file, click the delete icon.





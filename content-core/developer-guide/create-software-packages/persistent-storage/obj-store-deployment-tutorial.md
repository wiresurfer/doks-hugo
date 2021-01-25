---
title: "Object Store Deployment Walkthrough"
description:
type: developer-guide
date: 2019-10-25T12:36:50+05:30
pre: "a. "
weight: 295
---
## Learning Objective
The walkthrough demonstrates how a user can deploy software to the cloud
and leverage rapyuta.io persistent storage. For this, the walkthrough
will use the popular opensource S3 compatible object store,
[MinIO](https://www.minio.io/).

## Create MinIO File Server Package
To add the ***MinIO File Server*** package, follow the below steps
or click the below button:

{{< importpackage manifest="https://raw.githubusercontent.com/rapyuta-robotics/io_tutorials/master/io_manifests/minio_file_server_manifest.json" >}}

1. On the left navigation bar, Click **CATALOG**
2. Click **ADD NEW PACKAGE**.
3. The name of the package is `MinIO File Server`
4. The version of the package is set to **1.0.0** by default. 
5. Ensure **Is a singleton package** is ***not selected***.
6. Ensure **Is a bindable package** is ***selected***.
7. Write a summary of the package in the **Description** box. The package stores images, videos, and other unstructured data files in an object-store.
8. Click **NEXT**.
9.  The name of the component is `MinIO_FS`
10. Select **Cloud** as **Component Runtime**.
11. De-select **Is ROS Component**.
12. Ensure the value of **Replicas to run the component** is set to **1**
13. The name of the executable is `minio_executable`
14. Select **Docker** for **Executable Type**.
15. In the **Docker image** box, enter `rrdockerhub/minio-server`
16. To expose the file server application publicly over the internet, create a network endpoint by clicking **Add Endpoint**.
	1. The name for the endpoint is `FileStorage`
	2. Select **Exposed externally** checkbox.
	3. Select **HTTPS/WSS** for **Protocol**.
	4. In the **Port** box, the default value is `443`, and it cannot be modified.
	5. In the **Target Port** box, enter `9000`
17. To restrict access to the publicly available file server application, add the below **CONFIGURATION PARAMETERS**: *MINIO_SECRET_KEY* and *MINIO_ACCESS_KEY*.
	6. Click **Add Parameter**.
	7. In the **Name** box, enter `MINIO_SECRET_KEY`
	8. Similarly, add another parameter `MINIO_ACCESS_KEY`
18. Click **NEXT** > **CONFIRM PACKAGE CREATION**.

## Deploy MinIO File Server Package

#### Without Persistent Volume
Create transient storage by deploying the MinIO file server
without adding a persistent volume to it. The transient storage is
lost on intermittent failures, or when the deployment is stopped.
So, the data saved in temporary storage is permanently lost.
This example is only for testing purposes and gives no guarantee of your data.

To deploy ***MinIO File Server*** package without persistent volume,
follow the below steps:

1. Click **CATALOG** > Select **MinIO File Server** package > Click **Deploy package**.
2. The **Name of deployment** is `No Data Permanence`
3. Under **COMPONENT DETAILS**, the **Parameters** for **MinIO_FS (Cloud runtime)** are:
	1. In **MINIO_SECRET_KEY** box, enter a value for the secret key say `secretphrase`, and the secret key must be between 8 and 40 characters.
	2. In the **MINIO_ACCESS_KEY** box, enter a value for the access key say `accesskey` and ensure the access key must be at least three characters.
4. Click **CREATE DEPLOYMENT** > **Confirm**.

You will be redirected to the newly created deployment's **Details** tab. When the green-colored bar moves from **In progress** to **Succeeded**, it indicates that the **DEPLOYMENT PHASE** has **Succeeded** and the **STATUS** is **Running**. Then, the package is deployed successfully.

![No Data Permanence Deployment](/images/chapters/walkthroughs/object-store/no-data-permanence.png?classes=border,shadow&width=40pc)

To access the object storage at the provided
endpoint, copy and paste the highlighted URL address in a
new browser tab.

![File Storage Endpoint](/images/chapters/walkthroughs/object-store/file-storage-endpoint.png?classes=border,shadow&width=50pc)

In the **Access Key** and **Secret Key** boxes, enter the same access key and secret key that you provided while creating the above deployment, respectively.

![MinIO Dashboard Sign In](/images/chapters/walkthroughs/object-store/MinIO-signin-page.png?classes=border,shadow&width=40pc)

You can now access the dashboard of the MinIO object store at the provided endpoint. You can add buckets where unstructured data files can be stored.

Since this deployment does not use a persistent volume, any files that you add on the server will be lost when you stop (or de-provision) the deployment of the package.

#### With Persistent Volume
To preserve data files saved on the file server, deploy a persistent
volume and add it to the file server.

To deploy the **Rapyuta IO Persistent Volume** package, follow the below instructions:

1. On the left navigation bar, click **CATALOG**.
2. Select the public package, **Rapyuta IO Persistent Volume**, from **Storage packages**.
3. Click **Deploy package**.
4. The **Name of deployment** is `Volume Storage`
5. Ensure the ***disk type*** parameter of the **volumeComponent** is **SSD**. It provisions an SSD for block storage.
6. Ensure the ***capacity*** parameter of the **volumeComponent** is **32GiB**. It refers to the size of block storage.
7. Click **CREATE DEPLOYMENT** > **Confirm**.

To add ***Volume Storage*** to an instance of the MinIO file server, follow the steps:

1. On the left navigation bar, click **CATALOG**.
2. Select **MinIO File Server** package > **Deploy package**.
3. The **Name of deployment** is `Data Permanence`
3. For **MINIO_SECRET_KEY** provide the value `secretphrase`, and for **MINIO_ACCESS_KEY** provide the value `accesskey` respectively.
4. Click **Add volume**.
5. Under **Deployment**, select the persistent volume deployment that you created, **Volume Storage**
6. Select the name of the cloud component that needs persistent volume from the **Applicable Component** drop-down list. In this example, choose the **MinIO_FS** component.
7. Mount the persistent volume on a mount path. Since MinIO stores data files at ***/data***, provide `/data` as the value of **Mount path**.
8. Click **CREATE DEPLOYMENT** > **Confirm**.

The corresponding dependency graph looks like:
![Dependency Graph](/images/chapters/walkthroughs/object-store/volume-dgraph.png?classes=border,shadow&width=40pc)

Thus, the object store will be deployed with a persistent volume
mounted at path ***/data***. The persistent volume enables the file
server to preserve data stored in it even if its deployment is stopped.
It means that the stored data will be retained even when the deployment
of the MinIO File Server package is de-provisioned and provisioned again
with the same persistent volume.

A couple of observations made are:

1. If **Data Permanence** deployment is stopped, the **Volume Storage** deployment will continue to remain deployed.
2. If both **Data Permanence** and **Volume Storage** are running, you cannot stop **Volume Storage** before stopping **Data Permanence** because the volume is in use actively.
3. The volume deployments of **Persistent Volume** package can be added to only one deployment at a given time.
4. The number of volumes attached to a component must be less than or equal to the sum of all the cores of all executables of a component. Learn more in [package internals](/developer-guide/create-software-packages/package-internals/#components).

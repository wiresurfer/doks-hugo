---
title: "Persistent Storage"
description:
type: developer-guide
date: 2019-10-25T12:36:16+05:30
pre: "5. "
weight: 290
---
Applications running on the cloud de-allocate
any resources consumed when they stop, scale down, or fail.
This implies that the working storage associated with them is ephemeral.
To get around this problem rapyuta.io provides a mechanism to consume persistent [block storage](https://en.wikipedia.org/wiki/Block-level_storage)
for your applications running in the cloud. This storage can be
associated with at most one running deployment at any given point
of time. A user is typically required to manage the lifecycle of
the application code independently from the associated storage.

The **Rapyuta IO Persistent Volume** is a storage package. A storage package is a public package which is available to all users out of the box. You cannot delete or modify storage packages, and they are available to every user.

Follow this [tutorial to setup a simple object store](/developer-guide/create-software-packages/persistent-storage/obj-store-deployment-tutorial/)

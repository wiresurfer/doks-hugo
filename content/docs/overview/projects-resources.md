---
title: "Projects and Resources"
description: "We describe how rapyuta.io defines resources and isolates them into projects for clear separation and ease of management."
lead: "We describe how rapyuta.io defines resources and isolates them into projects for clear separation and ease of management."
date: 2019-10-25T14:09:54+05:30
lastmod: 2020-10-13T15:21:01+02:00
draft: false
images: []
menu: 
  docs:
    parent: "overview"
weight: 110
toc: true
---

## rapyuta.io Resources
rapyuta.io uses the concept of *resources* to manage and represent a set
of physical assets (such as robot locations and computing infrastructure
living in data-centers) as well as more abstract entities like the user's
software binaries, services, or credentials.

## Accessing resources through services
In cloud computing, each of these physical and abstract entities is mapped
to certain services. These services provide access to the underlying
resources. When you develop your application on rapyuta.io, you mix
and match these services into combinations that provide the
infrastructure you need, and then add your code to enable the
scenarios you want to build.

Common examples of resources in rapyuta.io are packages, deployments,
devices, and builds. As the platform adds new features and supports
more use-cases, more resources, and services to interact with them
are added.

## Projects
Any rapyuta.io resource that you create, or allocate and use must belong
to a project. You can think of a project as the organizational unit
for what you are building. A project is made up of the settings,
configuration, and other metadata that describe your applications.
Resources within a single project can work together easily, for
example, by communicating through an internal network. The
resources that each project contains remain separate across project
boundaries; you can only interconnect them through an external
connection.

Each project has:

* A project name, which you provide.
* A unique project ID, which rapyuta.io provides.
* miscellaneous metadata: e.g., creator name, timestamp, etc.

You may create multiple projects and use them to organize rapyuta.io
resources.

{{% notice info %}}
Learn more about projects [here](/developer-guide/organise-resources/project/).
{{% /notice %}}
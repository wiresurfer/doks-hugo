---
title: "Composing Software"
description:
type: developer-guide
date: 2019-10-25T12:50:41+05:30
pre: "3. "
weight: 410
---
Complex robotics applications are often built using more than one
programming language, in varied development environments, and by
people from distinct domains. For example, the interactive user 
interface of an application is built using a JavaScript framework
while the device component of the application is developed in
C/C++ and navigation algorithms are implemented in Python or
Java. Therefore, all of the teams involved in creating the
application have disparate development cycles. As a result,
each team depends on the finished running instances of components
developed by other teams.

rapyuta.io supports workflows that allow the user to combine these
packages at build time or runtime to combine multiple packages
and deployments that depend on each other to make possible more
complex functionalities and behavior.
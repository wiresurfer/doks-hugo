---
title: "Package Catalog"
description:
type: developer-guide
date: 2019-10-25T17:46:54+09:00
pre: "1. "
weight: 274
---
### Package Catalog
The catalog serves as the portal in rapyuta.io to streamline the software lifecycle management allowing you to spawn deployments, create new packages, and maintain package versions and updates. Additionally, it features a built-in collection of storage, communication, and curated platform packages that you can put together to build your solution.

It is also responsible for implementing design patterns that allow a user to leverage powerful build-time or run-time semantics to combine multiple packages and compose new behavior.

#### Packages Categories
rapyuta.io divides all packages in _Catalog_ into following categories:

1. **Public packages**     
   rapyuta.io provides public packages to its users. They are available to
   every user for deployment. You cannot delete or clone public packages.
   	* **Storage packages**    
	  Packages that perform disk read/write operations, represent databases,
	  execute storage related operations and implement other common use cases
	  are usually categorised as storage packages. For example,
	  **Rapyuta IO Persistent Volume** is a storage package.
	* **Communication packages**    
	  Packages that help in multi-device communication within the same network
	  are communication packages. For example,
	  **Rapyuta IO Communication Broker** is a communication package.
2. **Default packages**    
   The packages that you add to rapyuta.io are grouped as default packages.
   You can delete and/or clone default packages.
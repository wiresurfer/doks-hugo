---
title: "Secrets"
description:
type: developer-guide
date: 2019-10-25T12:37:31+05:30
pre: "6. "
weight: 300
---
A secret is an object containing sensitive information or confidential
data such as a password, SSH private key, SSL certificate. It grants rapyuta.io access to private git repositories and private docker images so that the platform can build source code in private git repositories or use private docker images.

There are two types of secrets available for you on rapyuta.io:

1. [Docker secret](/developer-guide/create-software-packages/secrets/docker-registry/)
2. [Source secret](/developer-guide/create-software-packages/secrets/sourcecode-repository/)
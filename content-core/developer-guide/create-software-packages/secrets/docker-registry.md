---
title: "Docker Registry"
description:
type: developer-guide
date: 2019-10-25T12:37:54+05:30
pre: "a. "
weight: 305
---
When you want to use a docker image from your private (docker) registry,
you grant rapyuta.io access to your private registry via a docker secret.

The two types of docker pull secrets that you can create are:

1. Docker Hub
2. Private Registry

## Creating a docker secret
To create a docker secret for a private docker registry, follow the steps:

1. On the left navigation bar, click **SECRETS**.
2. Click **ADD NEW SECRET**.
3. Under **SELECT SECRET TYPE**, click **Docker secret**.
3. In the **Name** box, enter a name for the docker secret. For instance,
   you may enter the name _docker-secret-name_.    
   Make sure that the name should be no longer than 253 characters. It must
   consist of lower case alphanumeric characters or hyphen -, and it must begin
   and end with an alphanumeric character.
4. If your docker registry is [Docker Hub](https://hub.docker.com/),
   select **Dockerhub**. Skip to instruction 6.
5. Otherwise, if you intend to use a private (docker) registry, select
   **Private registry**. Provide the private (docker) registry URL in the
   **Registry Url** box. It is mandatory to provide the registry URL.
6. In the **Username** box, type in your docker username.
7. In the **Password** box, type in your docker password.    
   To determine your docker credentials for your private registry,
   read about [authorisation token for docker credentials](/developer-guide/create-software-packages/secrets/docker-registry/#authorization-token).
8. In the **Email** box, enter the valid email address associated with your
   docker registry.
9. Click **SUBMIT**.

## Authorization Token
When you create a docker pull secret for a private registry, rapyuta.io stores
your docker credentials (that is, username and password) in base64-encoded
format. This encoded data is the _authorisation token_ which gives access to
rapyuta.io to pull private docker images while deploying a package.

To determine your docker credentials for a private registry, run the following
instructions in sequence on the system you have logged in to docker:

1. Docker login process creates or updates `config.json` file. To display this
    file, run the  command:
    ```bash
    cat ~/.docker/config.json
    ```

    A sample `config.json` file will look like:
   ```bash
   {
       "auths":{
           "https://index.docker.io/v1/":{
               "auth":"c3r...ze2"
           }
       }
   }
   ```
   The value of `auth` entry is base64-encoded data, also called
   an _authorisation token_.
   If you use a docker credentials store, you will instead see `credsStore` entry with the name of the store as value. For example, a sample `config.json` file with `credsStore` entry would look like:
   ```bash
   {
       "auths":{
           "https://index.docker.io/v1/":{}
       },
       "credsStore": "osxkeychain"
   }
   ```
   You can find out the _authorisation token_ from the respective `credsStore`
   entry’s value. In this case, use `osxkeychain` value to figure out
   authorization token.
2. To convert _authorisation token_ to a readable format, execute the command:
   ```bash
   echo  "c3r...ze2" | base64  -d
   ```
3. The output consists of two parts separated by a colon **:** as shown below
   ```bash
   janedoe:xxxxxxxxxx
   ```
   The part to the left of **:** is your docker username, while the one on the
   right is your password.

rapyuta.io uses your docker pull secret during package deployment.

If you encounter the following deployment error,
```bash
DEP_E153 (Could not pull either the docker image or the built package artifact for the component on the cloud)
```

Ensure the docker username and password in a secret are correct.
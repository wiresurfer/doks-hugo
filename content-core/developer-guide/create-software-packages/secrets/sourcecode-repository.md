---
title: "Source Code Repository"
description:
type: developer-guide
date: 2019-10-25T12:38:19+05:30
pre: "b. "
weight: 310
---
You can create packages using the source code from a private git repository.
A source secret allows rapyuta.io to access a private git repository or a git
repository with self-assigned or untrusted SSL certificate.

{{% notice info %}}
The rapyuta.io platform uses git version 2.16.6.
{{% /notice %}}

{{% notice note %}}
While cloning a git repository, ensure that you provide the appropriate protocol (HTTP/HTTPS). The HTTP to HTTPS redirection does not work while cloning the repositories.
{{% /notice %}}

The two types of source secrets are:

1. **Basic authentication**    
   authenticate using either git access token or your git username and password.
2. **SSH authentication**    
   authenticate using private SSH key of a git repository.

## Creating source secret
To create a source secret for the private git repository whose source code you
want to use for creating a package, follow the steps:

1. On the left navigation bar, click **SECRETS**.
2. Click **ADD NEW SECRET**.
3. Under **SELECT SECRET TYPE**, click **Source secret**.
4. In the **Name** box, enter a name for the source secret.
   For instance, you may name the source secret as _source-secret-name_    
   Ensure that the name should be no greater than 253 characters. It must
   consist of lower case alphanumeric characters or hyphen -, and it must begin
   and end with an alphanumeric character.
5. You may grant access via either your git username and password or the
   repository's git access token. Select **Basic Authentication** from the
   **Authentication Type** drop-down list.
	1. For authenticating via your git username and password, select **Password**.
		1. In the **Username** box, enter your git username.
		2. In the **Password** box, enter your password.
        ![Basic auth via password](/images/core-concepts/source-secret/basicauth-password.png?classes=border,shadow&width=40pc)
	2. For authenticating via the repository's git access token, select **Token**
		1. In the **Token** box, provide the corresponding git access token
6. If you want to grant access via SSH key of your git repository, select **SSH
   Authentication** from the **Authentication Type** drop-down list.
	1. In the **SSH Key** box, provide the private SSH key of your git repository.
    ![SSH authentication](/images/core-concepts/source-secret/sshauth.png?classes=border,shadow&width=40pc) 
7. Click **Submit**.


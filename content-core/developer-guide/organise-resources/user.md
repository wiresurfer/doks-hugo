---
title: "User"
description:
type: developer-guide
date: 2019-10-24T11:28:56+05:30
pre: "2. "
weight: 215
---
A user is a *rapyuta.io resource* that maps to an individual developer
on the rapyuta.io platform with unique login credentials. 

A user can be either in **Invited** state or in **Activated** state. 
They may be an *admin* and are responsible for managing the organization.

## Invite User to Organization
The admin of an organization can invite users to rapyuta.io

To invite or add users to rapyuta.io, you will follow the below
instructions:

1. On the left navigation bar, click **ORGANIZATION**. The panel will
   display information such as the name of the organization, its official website, location and a list of users.
2. On **Users** tab, enter a valid email address of the user you want to
   invite to rapyuta.io.
3. Click **ADD USER**.
   ![Add user to organization](/images/getting-started/organization/add-usr-org.png?classes=border,shadow&width=50pc)

The newly invited user is added to the existing list of **USERS**.
Initially, the state of the user is **Invited**. The invited user
will receive an invitation email to join rapyuta.io.
![User in invited state](/images/getting-started/organization/user-is-invited.png?classes=border,shadow&width=50pc)

Once the invited user has registered and signed into rapyuta.io, their state
changes to **Activated**.
![User in activated state](/images/getting-started/organization/invited-user-signs-in.png?classes=border,shadow&width=50pc)

{{% notice note %}}
The admin of an organization has the privilege to view *Invited* users. All other users can view only *Activated*
users.
{{% /notice %}}

## Remove User from Organization
The admin of an organization can remove a user from rapyuta.io.
However, the admin can only remove **Invited** users
from rapyuta.io.

To delete or remove an **Invited** user from rapyuta.io,
you will follow the below instructions:

1. On the left navigation bar, click **ORGANIZATION**.
   The page will display information such as the name of your
   organization, its official website, location, and a list of users.
2. On **Users** tab, search for the user you want to remove.
   Also, ensure that the user is in **Invited** state.
3. Click **Remove**.

![Delete user in invited state](/images/getting-started/organization/delete-invited-usr.png?classes=border,shadow&width=50pc)

{{% notice note %}}
To remove an **Activated** user, please <a href="#" onclick="javascript:FreshWidget.show();">contact support</a>.
{{% /notice %}}
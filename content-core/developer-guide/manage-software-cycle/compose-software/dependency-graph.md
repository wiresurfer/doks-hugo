---
title: "Dependency Graph"
description:
type: developer-guide
date: 2019-10-25T12:52:17+05:30
pre: "c. "
weight: 425
---
A dependency graph is a set of deployments and the relationships between them. The rectangular node of the graph represents a deployment (running or non-running). For example, consider the dependency graph shown below. The *COMMAND CENTER* and *SIMULATOR* are running deployments. The weighted red arrow connecting the two deployments indicates the relationship between them.

![dependency graph without ROS details](/images/core-concepts/deployments/dgraph-wo-ros.png?classes=border,shadow&width=30pc)

There are three types of relationships:

* **Dependent Deployment**    
  A rectangular node that is highlighted in red color is the running deployment
  whose **Details** you are currently viewing. The arrow points from a deployment to the deployment(s) it depends on.
  For instance, in the above graph, *COMMAND CENTER* depends on *SIMULATOR*, that is, *SIMULATOR* is a
  dependency (or "dependent deployment") of *COMMAND CENTER*.
* **ROS topic publisher-subscriber**    
  A green arrow represents one ROS Topic. It points from the deployment providing (publishing) the topic
  to the deployment consuming (subscribing to) the topic.
  For example, *SIMULATOR* publishes */sim/sensors* and */sim/pose* while *COMMAND
  CENTER* subscribes to the topics.

  ![dependency graph with ROS details](/images/core-concepts/deployments/dgraph-with-ros-details.png?classes=border,shadow&width=30pc)

{{% notice info %}}
To view ROS details of a dependency graph, select **Show ROS Communication Details** checkbox.
{{% /notice %}}

* **ROS service request - reply**    
  A pink arrow represents one ROS Service. It points from the deployment providing the service
  to the deployment consuming (calling) the service.
  For instance, the *COMMAND CENTER* requests the */teleport_turtle* and
  */register_sim_turtle service* from the *SIMULATOR*.
* **ROS action client - server**    
  A yellow arrow represents one ROS Action. It points from the deployment providing the action (server)
  to the deployment consuming (client) the action. 
  In this example, the *TURTLE* package listens to the command /turtle_0/goto_action from *COMMAND CENTER*.

 ![Complete dependency graph](/images/core-concepts/deployments/complete-dgraph.png?classes=border,shadow&width=60pc)


 You may click anywhere near a dependency graph (ensure you click anywhere only
in the blue color highlighted checkbox) to enlarge or shrink the graph. Press
the *ESC* key to exit after adjusting the graph's size.
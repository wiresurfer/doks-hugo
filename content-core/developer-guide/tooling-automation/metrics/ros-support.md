---
title: "Metrics from ROS Topics"
description:
type: developer-guide
date: 2019-10-25T13:27:22+05:30
pre: "b. "
weight: 540
---
Once a device is onboarded on to rapyuta.io, deploy a package on it to get
the ROS Master up and running. Now, you can subscribe to the ROS topic
for collecting the metrics data published by the topic.
 
rapyuta.io supports primitive data types:

* boolean
* integer (8, 16, 32, 64-bit)
* floating-point (32, 64-bit)
* byte
* character
* string
* time and duration

{{% notice info %}}
It also supports ***user-defined*** or ***custom ROS message types***.
{{% /notice %}}

Nested ROS messages are flattened and are displayed as a dictionary.
For example, consider a nested message ***geometry_msgs/PoseArray***
as shown below:

```bash
{
    header: {
        frame_id: 'base_frame'
    },
    poses: [
        {
            position: {
                x: 1.0,
                y: 0.0,
                z: 0.0
            },
            orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        },
        {
            position: {
                x: 1.1,
                y: 0.0,
                z: 0.0
            },
            orientation:{
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        }
    ]
}
```

After flattening the above nested ROS message, it will look like:

```bash
{
    "poses_0_orientation_w":1,
    "poses_0_orientation_x":0,
    "poses_0_orientation_y":0,
    "poses_0_orientation_z":0,
    "poses_0_position_x":1,
    "poses_0_position_y":0,
    "poses_0_position_z":0,
    "poses_1_orientation_w":1,
    "poses_1_orientation_x":0,
    "poses_1_orientation_y":0,
    "poses_1_orientation_z":0,
    "poses_1_position_x":1.1,
    "poses_1_position_y":0,
    "poses_1_position_z":0,
}
```
Any other data type (like bytearray, empty ROS message type) is
not supported and will be silently ignored.

## Type Introspection and Changing Data Types
When you subscribe to a ROS topic-based metric with a certain rosmsg
type, rapyuta.io introspects the data being published to the topic
so to generate metric types. The generated metric types are implicitly
bound to the name of the ROS topic just like a given ROS topic is
implicitly bound to a specific rosmsg type.

If you change the rosmsg type of the ROS topic in the future,
subscribing to the metrics of the topic will fail.
For example, consider a ROS topic **/geography** is defined such that
the data published to it is of type **std_msgs/String**, but if a new
or different publisher attempts to publish data of type
**custom_msg/KeyValuePair** as shown below and as you subscribe to
**/geography**, you will be automatically unsubscribed with the
error message: ***Invalid message format for topic /geography***

```bash
{
    key:"continent",
    value:"arctic"
}
```
You can view the error logs when subscribing/unsubscribing to metrics of a device at the **Subscribe/Unsubscribe error logs** table.
![Metrics error logs](/images/chapters/developer-guide/tooling-automation/metrics/metrics-error-logs.png?classes=border,shadow&width=50pc)

If a ROS topic is composed of integer and string data types,
for instance, consider a message ***sensor_msgs/BatteryState***,

```bash
{
    header:{
        stamp:now,
        frame_id:"map"
    },
    location:"B1",
    serial_number:"AAA",
    capacity:4.5,
    charge:3,
    current:3,
    voltage:9
}
```

When you subscribe to the above ROS topic, only the integer
data will be subscribed to and collected while the string
data will be ignored.

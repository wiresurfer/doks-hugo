---
title: "Rosbridge Compatibility"
description:
type: dev-tutorials
pre: "3. "
weight: 102
---
## Rosbridge WebSocket Compatibility
All the requests to deployment endpoints on rapyuta.io goes through a secure proxy which routes the requests to different backends.
When running behind such a reverse proxy, `rosbridge_websocket` fails at the connection handshake when the port in the request header is different from the port the deployment is running on (e.g., the default `9090`).

To workaround this limitation,

1. An additional argument, `websocket_external_port` needs to be included in the launch spec and should be set to the port 80
2. The `rosbridge_suite` repo needs to be added as a git submodule into your workspace.

An example launch spec would look something like this:
```
<launch>
    <arg name="ws_port"    default="$(optenv WS_PORT 9090)" />
    <arg name="ws_address" default="$(optenv WS_ADDR 0.0.0.0)" />
    <arg name="websocket_external_port" default="80" />

    <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
        <arg name="port"    value="$(arg ws_port)" />
        <arg name="websocket_external_port"    value="80" />
        <arg name="address" value="$(arg ws_address)" />
    </include>
</launch>

```

The [command center](/quick-walkthrough/#create-command-center-package) package in the quick walkthrough can be used as a reference as to how to pass `websocket_external_port`.

Some things to note in the example command center package mentioned above:

1. The [PR](https://github.com/RobotWebTools/rosbridge_suite/pull/468) for the external port feature has not been released and packaged yet (as on 13/04/2020). To include this, the [`rosbridge_suite`](https://github.com/RobotWebTools/rosbridge_suite) repo (devel branch) is added as a [git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules) into the [io_tutorials](https://github.com/rapyuta-robotics/io_tutorials) repository. You can choose to add a different commit other than devel, as long as it includes the commits in the above linked PR. 
2. The launch file of the command center at `io_turtlesim/io_turtle_command_center/launch/command_center.launch` has an additional argument called `websocket_external_port` and should be set to 80 to workaround this bug.

**P.S.:** 
- This github PR and the issues linked under it, throws more light on the issue and the need for external port: [https://github.com/RobotWebTools/rosbridge_suite/pull/468] (https://github.com/RobotWebTools/rosbridge_suite/pull/468)

There are PRs pending to fix this issue in the upstream. This workaround will be there till the following PRs are merged.

- [PR1](https://github.com/crossbario/autobahn-python/pull/1378)
- [PR2](https://github.com/RobotWebTools/rosbridge_suite/pull/494)

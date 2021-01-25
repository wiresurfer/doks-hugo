---
title: "System Metrics"
description:
type: developer-guide
date: 2019-10-25T13:27:01+05:30
pre: "a. "
weight: 535
---
Subscribing to system metrics will display a graphical visualization
(like a graph with metrics data plotted along x-axis and time along the y-axis) of the metrics.

{{% notice info %}}
Read the [metrics reference](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/) to know the data fields included in each system metric.
{{% /notice %}}

## Network IO Interface
You can monitor and analyze real-time network performance metrics.
For instance, you can collect upload and download rate per second for all interfaces for the last hour. The [metrics reference](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/#network-io-interface) includes the kind of data measured in a network IO graph.

## Disk usage
Subscribing to disk usage metrics displays a graph that shows
the information on disk usage metrics like available disk space,
percentage of disk space used, etc. Read [more](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/#disk-usage) to know the kind of disk usage metrics that are collected.

## Disk IO
You can gather metrics about disk traffic and timing by subscribing
to the disk IO. Read [more](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/#disk-io) to know the kind of information collected by disk IO metric.

## Memory usage
You can collect and visualize device memory metrics by subscribing to
the memory usage system metric. To know more about the data fields associated with memory usage, read [here](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/#memory-usage).

## CPU load average
You can determine the percentage of CPU used by a user, process, or system by subscribing to the CPU load average metric. Read the [CPU load average reference](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/#cpu-load-average) to understand the several data measurements plotted in the graph.

## Wireless
You can monitor and analyze the signal strength of a wifi connection by subscribing to the wireless metric. This information is extracted from ***/proc/net/wireless*** when a device is connected to a wifi connection.
Read the [wireless reference](/developer-guide/tooling-automation/metrics/system_metrics/sys-metric-reference/#wireless) to understand what kind of data will be collected when it is subscribed.

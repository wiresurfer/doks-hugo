---
title: "System Metrics Reference"
description:
type: developer-guide
date: 2020-03-03T16:10:43+05:30
pre: "2. "
weight: 537
---
## Network IO interface
Some of the metrics data measured in a network IO interface graph are:

* ***bytes_sent*** is the total number of bytes sent by the network interface
* ***bytes_recv*** is the total number of bytes received by the network interface
* ***packets_sent*** is the total number of packets sent by the network interface
* ***packets_recv*** is the total number of packets received by the network interface
* ***error_in*** is the total number of receive errors detected by the network interface
* ***error_out*** is the total number of transmit errors detected by the network interface
* ***drop_in*** is the total number of received packets dropped by the network interface
* ***drop_out*** is the total number of transmitted packets dropped by the network interface

## Disk usage
The disk usage metrics data includes the following fields:

* ***free*** is the amount of disk space that is freely available. It is an integer value shown as byte.
* ***total*** is the total amount of disk space. It is an integer value shown as byte.
* ***used*** is the amount of disk space that is in use. It is an integer value shown as byte.
* ***used_percent*** is the percentage of disk space used. It is a floating-point value shown as percentage.
* ***inodes_free*** is the number of free inodes. It is an integer value shown as files.
* ***inodes_used*** is the number of used inodes. It is an integer value shown as files.
* ***inodes_total*** is the total number of inodes. It is an integer value shown as files.

## Disk IO
The following fields comprise the disk io metric:

* ***reads*** is a counter that increments when a read request completes. It is an integer value.
* ***writes*** is a counter that increments when a write request completes. It is an integer value.
* ***read_bytes*** is the count of the number of bytes read from the device. It is an integer value shown as byte.
* ***write_bytes*** is the count of the number of bytes written to the device. It is an integer value shown as byte.
* ***read_time*** is the count of the number of milliseconds that read requests have waited on the device. It is an integer value shown as milliseconds.
* ***write_time*** is the count of the number of milliseconds that write requests have waited on the device. It is an integer value shown as milliseconds.
* ***io_time*** is the count of the number of milliseconds during which the device has had IO requests queued. It is an integer value shown as milliseconds.
* ***weighted_io_time*** is the count of the number of milliseconds that IO requests have waited on the device.
* ***iops_in_progress*** is the count of the number of IO requests that have been issued to the device but have not yet completed. It does not include IO requests that are in the queue but not yet issued to the device.

## Memory usage
Some of the data collected by this metric includes:

* ***available*** is the amount of memory that is available. It is an integer value shown as byte
* ***available_percent*** is the percentage of memory that is available. It is a floating-point value shown as percentage
* ***buffered*** is the amount of physical RAM used as cache memory. It is an integer value shown as byte
* ***cached*** is the amount of physical RAM used as cache memory. It is an integer value shown as byte
* ***free*** is the amount of free RAM. It is an integer value shown as byte
* ***inactive*** is the amount of memory that hasn't been used in some way. It is an integer value shown as byte
* ***slab*** is the amount of memory used by the kernel to cache data structures for its use. It is an integer value shown as byte
* ***total*** is the total amount of physical RAM. It is an integer value shown as byte.
* ***used*** is the amount of RAM in use. It is an integer value shown as byte.
* ***used_percent*** is the percentage of memory currently in use. It is a floating-point value shown as percentage
* ***active*** is the amount of memory that has been used in some way. It is an integer value shown as byte.
* ***wired*** is the memory where the kernel and other low-level components like device drivers and virtual memory objects are stored.

## CPU load average
The following are the data measurements plotted in the graph viualization of CPU load average metric:

* ***usage_guest*** is the percentage of time that the CPU is running a virtual CPU for a guest operating system
* ***usage_guest_nice*** is the percentage of time that the CPU is running a virtual CPU for a guest operating system, which is low priority and can be interrupted by other processes.
* ***usage_idle*** is the percentage of time that the CPU is idle
* ***usage_iowait*** is the percentage of time that the CPU is waiting for IO operations to complete
* ***usage_irq*** is the percentage of time that the CPU is servicing interrupts
* ***usage_nice*** is the percentage of time that the CPU is in user mode with a low priority process, which a higher priority process can interrupt
* ***usage_softirq*** is the percentage of time that the CPU is servicing software interrupts 
* ***usage_steal*** is the percentage of time that the CPU is in stolen time or time spent in other operating systems in a virtualized environment
* ***usage_system*** is the percentage of time that the CPU is in system mode
* ***usage_user*** is the percentage of time that the CPU is in user mode

## Wireless
The wireless statistics are device-dependent. Each driver will provide
only some statistics based on hardware support. As the values depend on the driver, the range of values may change. Hence, you need to refer to your driver documentation for the correct interpretation of those values. rapyuta.io extractss wireless metric data from ***/proc/net/wireless***.
For more information, read the manual page of [iwconfig](https://linux.die.net/man/8/iwconfig).
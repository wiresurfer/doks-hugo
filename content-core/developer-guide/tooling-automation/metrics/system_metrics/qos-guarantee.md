---
title: "QOS Guarantee"
description:
type: developer-guide
date: 2020-03-03T16:14:19+05:30
pre: "1. "
weight: 536
---
The QoS guarantee level is an agreement between the sender of a message and the receiver of a message that defines the guarantee of delivery for a specific message. In rapyuta.io, the messages are generally metrics or logs of a device, which are sent to rapyuta.io servers.

{{% notice note %}}
QoS guarantee levels described here are ***different*** from the QoS guarantee that you set while adding a new rapyuta.io package.
{{% /notice %}}

There are three QoS levels:

* Low
* Medium
* High

#### Low QoS guarantee
The minimum QoS level is *Low*. This level guarantees best-effort
delivery, but delivers with some data loss. Use Low QoS level when you
have a completely or mostly stable connection between sender and receiver,
or if the loss of data (metrics or logs)  is acceptable.
It is suggested to set the QoS level to low when the
rate of data generation is high to reduce the overhead cost and
lower the latency while sending messages.

#### Medium QoS guarantee
*Medium* QoS level guarantees that metrics or logs are delivered at least once to the receiver. It is possible for the data to be delivered multiple times. Use Medium QoS level when you need to get every data, and your application can tolerate duplicates and be able to process them accordingly.

#### High QoS guarantee
The maximum QoS level is *High*. This level guarantees that each metric
or log is received only once by the receiver. It is the safest and
slowest QoS level. Use High QoS level if it is critical to your
application to receive all data exactly once or if a duplicate
delivery can harm your application users or subscribing clients.
Selecting a high QoS level when the rate of data generation is high
will lead to high overhead cost and high latency while sending
messages, which you might not want to do.
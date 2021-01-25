---
title: "Pricing Calculation"
description:
type: pricing-support
date: 2019-10-24T14:00:41+05:30
pre: "3. "
weight: 720
---
This pricing example is based on the **Professional Plan**
subscription.

Let’s assume your ROS software application has two modules say
*A* and *B*. You intend to deploy three instances of
A in the cloud, an instance of B on your device, and attach
a *persistent storage volume of 32GiB size* to your application.
You also want the ROS application to run for 10 hours a day.

In rapyuta.io, the ROS software application is modeled as a
ROS *package*. The application modules are implemented as
*components* of the package, and each component has at least
one *executable* (a runnable entity).
The instances of a component are called *replicas*.

You will create a ROS package with two components say *compA* and
*compB* using rapyuta.io.

* Let's assume *compA* will have two executables that actually
  get deployed when *compA* is deployed in the cloud.
  So, *compA* is charged based on cloud deployment hours.
  The compute and memory values for one of the executables
  are (1 cpu core, 4GiB memory) while the other executable has (2 cpu core, 8GiB memory).
  You will create 3 replicas of *compA*.
* Suppose *compB* will have a single executable that gets
  deployed when *compB* is deployed on a device.
* You will deploy a persistent storage volume of 32GiB size,
  which will be used by a deployment of the ROS package.
  So, this persistent volume deployment is charged for
  volume deployment hours.

You will then deploy the ROS package and have it run for 10 hours
a day.

***Monthly charges*** will be calculated as follows:

Cloud deployment hours (CDH) price per vCPU-hour is $0.10<br>
CDH charges for all executables of an instance of *compA*
running for 10 hours a day for 30 days is <br>**(CDH price * (sum of vCPUs of all executables) * hours)**: $0.10 * (1+2) * 10 * 30 days = $90<br>
CDH charges for 3 copies of compA: $90 * 3 = ***$270***

Volume deployment hours (VDH) price per GiB-hour is $0.00025<br>
VDH charges for a storage volume of 32GiB size running for
10 hours a day for 30 days is <br>**(VDH price * (storage volume size in GiB) * hours)**:
$0.00025 * 32 * 10 * 30 = ***$2.4***

Total pay as you use charges: $270 + $2.4 = **$272.4**

Let’s assume you may require more add-ons, for example, you
add 5 devices.

Add-ons price per device per month is $100<br>
Add-ons price for 5 devices per month: $100 * 5 = **$500**

Total charges: $272.4 + $500 = **$772.4**

### Add-ons Pricing Calculation
The add-ons (devices, users and static-routes) are charged differently
from the deployment hours.

For example, let's assume you have subscribed to one of the plans on
March 1. Suppose you add 5 additional devices on March 10. You will
be charged for those 5 devices. Now, going further, you may experience
either of the following three cases:

#### Case 1
Suppose you add 3 *devices add-on* on March 20. You will be charged for
these devices. So, the total number of devices added till now is 8
(5 + 3), and an overage charge for 8 devices is reflected in the final
bill.

#### Case 2
Suppose you remove 2 *devices add-on* on March 20.
The *devices add-on* count will immediately decrease to 3.
You will be charged for 5 devices (that were added on March 10)
in the current bill. The removed devices will not be charged in
the next month's bill.

#### Case 3
Suppose you neither add nor remove any *devices add-on* after March 10. You will be charged for the 5 *devices add-on* that you
added on March 10.

Your credit card will, eventually, be charged with the final bill amount
on the first day of the next month.
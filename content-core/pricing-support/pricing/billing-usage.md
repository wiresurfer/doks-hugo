---
title: "Billing and Usage"
description:
type: pricing-support
date: 2019-10-24T14:00:31+05:30
pre: "2. "
weight: 715
---
## Consumable Items
Each subscription plan in rapyuta.io includes a predefined number of complementary consumable resources or units such as cloud deployment hours, volume deployment hours, devices, users, and static routes. To cater to different user requirements rapyuta.io allows you to purchase specific add-on units (up to predefined maximum caps for the plan) at an additional cost.

![Billing Usage](/images/pricing/billing/billing-usage.png?classes=border,shadow&width=50pc)

Overage charges will be incurred for each resource in the instance where the included quota corresponding to that resource in the subscribed plan is exhausted. These additional charges are levied only for what you use.

* The ***Current Usage*** is the count of units currently being used.
* The ***Included Units*** is the count of units included in the subscribed plan.
* The ***Max Units*** is the count of maximum units available in the subscribed plan.
* The ***Add-Ons*** is the current count of add-ons you have purchased.

{{% notice info %}}
In the case of volume deployment hours, rapyuta.io provides SSD disks as persistent storage volumes. The disks are of **32GiB**, **64GiB**, **128GiB**, **256GiB** and **512GiB** sizes. There also exists a cap on the total allocated quota of volumes and total disk space per account which is **10 volumes / 1TiB** and **20 volumes / 2TiB** size for community and professional plans respectively.
{{% /notice %}}

#### Additional Consumable Resources
You can augment the number of allocated consumable resources in a subscription plan to suit your needs by purchasing the required
prepaid add-ons. The add-on units augment the allowed resource limits against your subscription and are charged on a monthly basis.

##### Add or remove add-ons

1. Click **Add/Remove add-ons** adjacent to the add-on name.
   ![Add device add-on](/images/pricing/billing/add-route-addon.png?classes=border,shadow&width=50pc)
2. Increase or decrease the count of static routes by using the arrows.
3. Click **UPDATE**.
   ![Add-ons](/images/pricing/billing/increase-addon-count.png?classes=border,shadow&width=50pc)

## Billing
On subscribing to one of the plans, you (the admin of organization)
can access the **Billing** panel.
*Billing* aggregates information on the charges incurred on
consumable units, and displays it in tabular and graphical forms.

The subscription fee is recurring and prepaid. The credit card will be charged monthly on the first day of the month. It means the payment receipt is generated at the start of the month. You will receive an email with the payment receipt at the start of every month. The payment receipt shows the subscription fee of the current month and usage charges of the previous month.

![Billing](/images/pricing/billing/billing-chart.png?classes=border,shadow&width=50pc)

* The **ESTIMATED BILL** is calculated based on the estimated usage for the month.
* The **NEXT BILLING DATE** is the date when the credit card will be charged towards the payment of the final bill including taxes.
* The **PLAN** displays the plan you have subscribed to.
* The pie chart provides you insights into the relative component of any
  *additional usage charges* and *standard subscription charges*. Additional usage indicates the count of additional devices, users, cloud deployment hours and volume deployment hours.

## Payment Method
The admin of an organization can **add multiple credit cards** to the organization's billing account. But, only one credit card is made the **primary** or **default** card, which is used to charge the subscription fee and additional amount for using rapyuta.io resources.

You can **delete any existing credit card** except the primary card because it is used for monetary transactions. If you want to delete the primary credit card, ensure you make another card a primary card before deleting the previous one.

{{% notice note %}}
If you want to unsubscribe, or cancel the current plan, or upgrade to another plan,
please <a href="#" onclick="javascript:FreshWidget.show();">contact support</a>.
{{% /notice %}}
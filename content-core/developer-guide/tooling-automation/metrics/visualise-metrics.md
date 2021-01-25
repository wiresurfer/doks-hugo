---
title: "Metrics Visualization"
description:
type: developer-guide
date: 2019-10-30T16:28:20+05:30
pre: "c. "
weight: 542
---
In this quick how-to guide on visualizing metrics from a ROS topic, you will
publish data to a topic in rate mode by executing the command below on
the device.

```bash
rostopic pub -r 1 /random std_msgs/Float32 5.0
```

1. Click **DEVICES** > select the device which is publishing ***/random*** topic > click **Metrics**.
2. To visualize the data being published by a ROS topic, you will have to first subscribe to the ROS topic by clicking **Subscribe**. In this example, subscribe to the ***/random*** ROS topic.
![Subscribe topic](/images/chapters/developer-guide/tooling-automation/metrics/click-subscribe.png?classes=border,shadow&width=50pc)
![Subscribed topic](/images/chapters/developer-guide/tooling-automation/metrics/subscribe-rostopic.png?classes=border,shadow&width=50pc)
1. To add a new visualization, click **Add Cell**.
![Add Cell](/images/getting-started/add-cell.png?classes=border,shadow&width=70pc)
1. Click **Add a Query**.
![Add Query](/images/getting-started/add-query.png?classes=border,shadow&width=70pc)
5. Select the ROS topic whose metrics need to be plotted on a graph. In this example, select the ***/random*** topic.
![Select topic](/images/chapters/developer-guide/tooling-automation/metrics/select-random.png?classes=border,shadow&width=50pc)
6. Select the topic's field like **data** in this example.
![Select Data](/images/chapters/developer-guide/tooling-automation/metrics/select-data.png?classes=border,shadow&width=50pc)
7. You may plot multiple functions such as mean, sum, median in the same graph. Click on **1Function** button to open the list of fuctions available. Select the functions you want to visualize on the graph like *mean* in this example, and then click **Apply**.
![Plot Function](/images/chapters/developer-guide/tooling-automation/metrics/select-function.png?classes=border,shadow&width=50pc)
8. You may change the type of the graph by selecting one of the available graph forms like ***line***, ***stacked***, ***step-plot*** or ***bar***. In this example, a **Bar** graph is selected. To confirm the current settings, click the red colored checkmark at the top-right corner.
![Bar graph](/images/chapters/developer-guide/tooling-automation/metrics/bar-graph.png?classes=border,shadow&width=50pc)
9. To edit the ***title*** of the graph, click the pencil icon at the top right corner of the cell as indicated by 1 in the below image.
10. To modify the ***current graph settings***, click the wheel icon at the top right corner of the cell as indicated by 2 in the below image.
11. To ***delete*** the cell, click on the cross mark at the top right corner of the cell as indicated by 3 in the below picture.
![Graph Window Options](/images/getting-started/graph-window.png?classes=border,shadow&width=70pc)
12. To change the refresh interval, click **Every 60 seconds** drop-down list and select your desired interval.
13. To change the time range, click **Past 5 minutes** drop-down list and select your desired range.
14. The final graph may look like the one below:
![Final graph](/images/chapters/developer-guide/tooling-automation/metrics/graph-1.png?classes=border,shadow&width=50pc)


{{% notice note %}}
The procedure to visualize the system metrics is the same as
the procedure to visualize metrics from a ROS topic as explained
above.
{{% /notice %}}
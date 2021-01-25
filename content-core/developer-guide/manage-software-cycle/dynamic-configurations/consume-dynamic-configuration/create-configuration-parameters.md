---
title: "Creating a Dynamic Configuration"
description:
type: developer-guide
date: 2019-10-30T15:06:45+05:30
pre: "b. "
weight: 403
---


***rapyuta.io*** allows you to create a dynamic configuration that holds the parameter files into a tree-like hierarchical structure called a *configuration hierarchy*. These configuration parameters are applied to the robots to achive the desired behavior for the robot. Perform the following procedure to create a dynamic configuration.


1. Log on to ***rapyuta.io*** console and on the left navigation bar, 
click **CONFIGURATIONS > ADD NEW CONFIGURATION**.
2. In the **Add new configuration** pop-up window,
 type a name for the configuration and click **Confirm**.
The new configuration name is added to the **Configuration Name** list. 
To configure the root node (configuration name), click the configuration name.
3. Hover over the root node. It allows you to create an attribute node 
and base configuration file.
![YAML file](/images/core-concepts/configurations/root-node-actions.png?classes=border,shadow&width=25pc)

 {{% notice info %}}
You can add only one attribute node but multiple base configuration files 
from the root node.
   {{% /notice %}}
4. Click **Add file**. The **Create new file** window is displayed.
    In the **Create new file** window, do the following.<br/>
5. From the File type drop-down menu, select either YAML or Binary file.<br/>
6. If you have selected the file type as YAML,
 in the File name field, type a name for the file and click **Confirm**. 
 You can add the configuration in the File Contents area. 
 For more information on how to add and edit the configuration in YAML format, see [Rules for writing/editing configuration parameters files.](#rules-for-writing-yaml-configuration-parameters) To see an example, [click here](#creating-a-configuration-parameter-in-yaml-file-format). 
![YAML file](/images/core-concepts/configurations/yaml-file.png?classes=border,shadow&width=30pc)
7. If you have selected the file type as Binary, in the File type field, you can upload or drag a binary configuration file from the local machine. To see an example, [click here](#creating-a-configuration-parameter-in-binary-file-format).
![Binary file](/images/core-concepts/configurations/binary-file.png?classes=border,shadow&width=30pc)
8. Click **Confirm**.

The configuration parameter hierarchy is created. You can apply the configurations to a device now. For more information, see [Applying Dynamic Configurations](/developer-guide/manage-software-cycle/dynamic-configurations/consume-dynamic-configuration/apply-dynamic-configs)




#### Rules for writing YAML configuration parameters
There are a set of rules that you must adhere to when writing a configuration
parameters file. They are:

- A space character always follows a colon (:) and a hyphen (-). For instance,
   ```yaml
   # Invalid parameters
   a:b
   list:
        -a
        -b
    # Valid parameters
    a: b
    list:
        - a
        - b
   ```
-  Start your configuration parameters file with maps.
   ```yaml
   # Invalid parameter file example
   abc      # string
   # Invalid parameter file example
   [a, b]   # list
   # Valid parameter file example
   a:
        b: c
        d: e
        f:
           - g
           - h
   ```
-  If you add a new item to a default list parameter, it is appended to the list.
   ```yaml
   # base parameters file
   list:
        - a
        - b
    # extending parameters file
    list:
        - c
    # merged resultant file
    list:
        - a
        - b
        - c
   ```
-  If you modify a default parameter's value, the new value overrides the old value.
   ```yaml
   # base parameters file
   a: b
   # overriding parameters file
   a: e
   # merged resultant file
   a: e
   ```

## Creating a configuration parameter in YAML file format ##

Rapyuta.io allows you to add the configuration parameters at every value node including the root node. When you hover and click the value node to add a configuration file, you get an option to choose to write the configuration parameter in YAML format or upload the configurations directly from your local machine.

In the following procedure, the configuraion parameter for a RGV robot is written in YAML format.

1. Log on to ***rapyuta.io*** console and on the left navigation bar, click **CONFIGURATIONS > ADD NEW CONFIGURATION**.

2. In the **Add new configuration** pop-up window, type a name for the configuration and click **Confirm**. For this example, type the name as ***config_1***.
The new configuration name is added to the **Configuration Name** list. To configure the root node (configuration name), click **Config_1** from the list.
3. Hover over **Config_1** and add an attribute as **country**.

4. Hover over **Config_1** and click **Add file**. By default, the file type is selected as YAML.Type the file name as  **RGV_config** and click **Confirm**.  
   
5. In the **File Contents** area , type the RGV robot parameter values, as displayed in the following figure and click **Save**.
{{% notice info %}}
Ensure you read the
[rules for writing/editing configuration parameters files](#rules-for-writing-yaml-configuration-parameters).
{{% /notice %}}
    ![base parameters](/images/core-concepts/configurations/parameter-defaults.png?classes=border,shadow&width=40pc)

6. Hover over the ***country*** attribute to view options to add new
    value nodes.
7.  Click **Add country value**. Give it a name, for example, ***USA***.
    Click **CONFIRM**. Similarly, add another country value and give it the name ***Japan***.

8.  Suppose that the manufacturer *Tesla* has signed the contract deal
    to manufacture the RGVs in the USA region. You can add *Tesla* to
    the existing list of ***allowed_manufacturers***. You will have to
    define ***RGV_Config.yaml*** under the ***USA*** value, which will extend the
    list of base parameters. The resultant file after merging the
    base parameters in **config_1/RGV_config.yaml** and the newly added parameters in
    **USA/RGV_config.yaml** will include **tesla** in addition to those already present.
    ![sample file of USA](/images/core-concepts/configurations/USA-sample.png?classes=border,shadow&width=65pc)
9.  Suppose that the regulation in Japan requires you to limit the
    maximum velocity of a RGV from *5m/s* to *3m/s*. You can override
    the ***max_velocity*** of the RGV by assigning a new value to it. You
    will have to define ***sample.yaml*** file under the ***Japan*** value,
    which will include the ***max_velocity*** parameter, but with its default
    overridden. The final parameters file is a result of merging the base parameters (**config_1/RGV_config.yaml**) and the overridden parameters (**Japan/RGV_config.yaml**).
    ![sample file of Japan](/images/core-concepts/configurations/japan-sample.png?classes=border,shadow&width=65pc)

## Creating a configuration parameter in Binary file format ##

A binary file can be of any format, for example, .png, .json, .txt, .jpg etc. ***rapyuta.io*** allows you to upload the configuration file as a binary file. You can upload the configuration file if the configuration file cannot be written in YAML format, for example a map image.

In the following example, a configuration hirarchy is created with the binary configuration files.

1. Log on to ***rapyuta.io*** console and on the left navigation bar, click **CONFIGURATIONS > ADD NEW CONFIGURATION**.

2. In the **Add new configuration** pop-up window, type a name for the configuration and click **Confirm**. For this example, type the name as ***config_2***.
The new configuration name is added to the **Configuration Name** list. To configure the root node (configuration name), click **Config_2** from the list.
3. Hover over **Config_2** and add an attribute as **country**.
4. Hover over **Config_2** and click **Add file**. 
5. Upload a base map image as the binary file from your local machine and click **Confirm**. If no configuration binary file is added at the file node, the configuration of base map image is applied to the device.
6. Hover over the root node again to view options to add a new attribute node.
7. Click Add attribute. Give it a name, say, country. Click CONFIRM.
8. Hover over the country attribute to view options to add new value nodes.
9. Click Add country value. Give it a name, for example, ***USA***. Click **CONFIRM**. Similarly, add another country value and give it the name Japan. 

![sample image file](/images/getting-started/apply-config-paramas/binary-file.png?classes=border,shadow&width=20pc)
10. For the attribute value ***USA***, add the binary configuration file from your local machine and click Confirm.(For this example the map image is warehouse_map ). Similarly you can upload a binary parameter file for the attribute value japan.
The binary configuration parameter values are saved. The configuration will be applied to the devices after you apply the configuration. For more information, see [Apply configuration parameters] (/developer-guide/manage-software-cycle/dynamic-configurations/consume-dynamic-configuration/apply-dynamic-configs).



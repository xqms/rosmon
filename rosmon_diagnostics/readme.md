# rosmon_diagnostic
This node convert the [State message](../rosmon_msgs/State.msg) into a ROS [diagnostic_msgs/DiagnosticArray](http://docs.ros.org/melodic/api/diagnostic_msgs/html/msg/DiagnosticArray.html).
This allow to have processes's state information into the robot diagnostics system.

# configuration
This node has several parameters, have a look to the following example :
```xml
<node pkg="rosmon_diagnostics" type="rosmon_diagnostics_node" name="rosmon_diagnostics">
	<param name="diagnostics_prefix" value="procs_"/>
	<remap from="rosmon_state" to="/my_rosmon/state"/>
	<rosparam>
defaultMemoryLimit_byte: '150 kB'
defaultCPULimit: 0.2
memory_limits: {foo_node: '250 kB', bar_node: '4.5 MB'}
cpu_limits: {foo_node: 0.6}
    </rosparam>
</node>
```

## Parameters :
* ```diagnostics_prefix``` : This prefix will be added to all diagnostics name. Diagnostic name will be diagnostics_prefix + node_name. This is usefull when using the Diagnostics Aggregator to sort diagnostics. Look at [this launch file](./test/tester.launch) and [this diagnostics aggregator configuration file](./test/analyzers.yaml) to better understand.
* ```defaultMemoryLimit_byte```: If a node use more memory that this parameter, corresponding diagnostics will pass in "WARNING". *Default value is 15 MB*.
* ```defaultCPULimit``` : If user cpu + system cpu is greater than this value,  corresponding diagnostics will pass in "WARNING". This is the same unit as in the rosmon message. *Default value is 0.05*. 
* ```memory_limits``` : A dictionnary in which keys are node names, and values are memory limits. This parameter allow to override the ```defaultMemoryLimit_byte``` for a single node. *Default value is empty*
* ```cpu_limits``` : A dictionnary in which keys are node names, and values are CPU limit. This parameter allow to override the ```defaultCPULimit``` for a single node. *Default value is empty*

## Memory bytes parser :
A built in memory byte parser allow user to enter memory limits in a human readable form. Parser understand the following memory suffix : B, kB, KB, mB, MB, gB, GB, tB, TB for respectively Byte, Kilobytes, Megabytes, etc ...

## Subscribed values
This node subscribe to a [State message](../rosmon_msgs/State.msg) from rosmon. You will have to remap the topic ```rosmon_state``` to the topic published by rosmon.
If you launch rosmon with the standard command ```mon launch <my launch file>```, the rosmon node will have an anonymized name.
So you will not be able to know the topic name.

You must launch rosmon with the ```--name``` argument to know the rosmon node names, and so be able to remap the topic.
In the previous launch file example, rosmon have been launched with :
 
```mon launch --name my_rosmon <my launchfile>```
 
## Published values
This node publish a [diagnostic_msgs/DiagnosticArray](http://docs.ros.org/melodic/api/diagnostic_msgs/html/msg/DiagnosticArray.html) on the ```/diagnostics``` message.
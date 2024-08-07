# Ros2基本操作

> ref: https://docs.ros.org/en/galactic/Tutorials/

source in every terminal

```
source /opt/ros/galactic/setup.bash
```



根据教程玩一下turtlesim

##### ros2 run

```
ros2 run <package_name> <executable_name>
```

##### ros2 node list

```
ros2 node list
```

查看topic数据

```
ros2 topic echo <topic_name>
```

查看topic 信息

```
ros2 topic info <topic_name>
```

```
ros2 interface show <topic_type>
```



ros2发布topic数据

```
ros2 topic pub --rate <publish_freq> <topic_name> <topic_type> data
#rate后是发送频率，1为1hz
e.g. ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

ros2 topic pub --rate 1 /vehicle/status/steering_status autoware_auto_vehicle_msgs/msg/SteeringReport "{steering_tire_angle: 0.5}"

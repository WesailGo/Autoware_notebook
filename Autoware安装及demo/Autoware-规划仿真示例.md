# Autoware 规划仿真示例

> ref: https://autowarefoundation.github.io/autoware-documentation/galactic/tutorials/ad-hoc-simulation/planning-simulation/

下载地图

```shell
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
```

sudo apt-get install ros-galactic-ros2bag ros-galactic-rosbag2*

## Lane Driving Scenario

#### 启动autoware

```shell
source ~/work/autoware/install/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

<img src="./Autoware-%E8%A7%84%E5%88%92%E4%BB%BF%E7%9C%9F%E7%A4%BA%E4%BE%8B_imgs/image-20240731170916717.png" alt="image-20240731170916717" style="zoom:67%;" />

启动成功

#### 添加panel

左上角click `Panels -> Add new panel`, select `AutowareStatePanel`, and then click `OK`.

#### 添加初始pose

- Click the `2D Pose estimate` button in the toolbar, or hit the `P` key.

  <img src="./Autoware-%E8%A7%84%E5%88%92%E4%BB%BF%E7%9C%9F%E7%A4%BA%E4%BE%8B_imgs/image-20240731171618673.png" alt="image-20240731171618673" style="zoom:33%;" />

- In the 3D View pane, click and hold the left-mouse button, and  then drag to set the direction for the initial pose. An image  representing the vehicle should now be displayed.

  <img src="./Autoware-%E8%A7%84%E5%88%92%E4%BB%BF%E7%9C%9F%E7%A4%BA%E4%BE%8B_imgs/image-20240731171749115.png" alt="image-20240731171749115" style="zoom:50%;" />

操作完3D视图里就出现了车

#### 添加目标pose

- Click the `2D Goal Pose` button in the toolbar, or hit the `G` key.、

  <img src="./Autoware-%E8%A7%84%E5%88%92%E4%BB%BF%E7%9C%9F%E7%A4%BA%E4%BE%8B_imgs/image-20240731172001540.png" alt="image-20240731172001540" style="zoom:50%;" />

- In the 3D View pane, click and hold the left-mouse button, and  then drag to set the direction for the goal pose. If done correctly, you will see a planned path from initial pose to goal pose.

  ![image-20240731172125160](./Autoware-%E8%A7%84%E5%88%92%E4%BB%BF%E7%9C%9F%E7%A4%BA%E4%BE%8B_imgs/image-20240731172125160.png)

#### 启动小汽车

两种方式：

- 在可视化界面上点击engage按钮

- ```shell
  source ~/work/autoware/install/setup.bash
  
  ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
  ```

两种方式都可以看到小车在3d视图中按规划路径动起来了



## Parking Scenario

在原本rviz界面中

![image-20240731172925199](./Autoware-%E8%A7%84%E5%88%92%E4%BB%BF%E7%9C%9F%E7%A4%BA%E4%BE%8B_imgs/image-20240731172925199.png)

相同的操作逻辑，只是改变起点与终点，在到达时会从lane driving mode 转为 parking mode
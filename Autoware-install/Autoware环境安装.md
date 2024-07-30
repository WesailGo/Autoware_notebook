# Autoware环境安装

> Ref: 
>
> - https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/
> - https://blog.csdn.net/zardforever123/article/details/132029636

### 环境配置：

- Ubuntu20.04
- Nvidia A4000

***目的： 创建完成一个workspace***

## 配置Nvidia驱动

自己的工作站并没有下载安装Nvidia驱动，所以需要先自行下载

在命令行输入

```
$ ubuntu-drivers devices
```

<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240729151328452.png" alt="image-20240729151328452" style="zoom:50%;" />

找到recommended，接下来去 软件与更新---->附加驱动 选择相应的驱动，点击应用更改，重启电脑即可，无需禁用原驱动（会导致黑屏）

<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240729151515701.png" alt="image-20240729151515701" style="zoom:50%;" />



## 克隆Autoware仓库

```
$ git clone https://github.com/autowarefoundation/autoware.git -b galactic
$ cd autoware
```

## 安装CUDA、Cudnn、TensorRT

cuda 下载链接：https://developer.nvidia.com/cuda-toolkit-archive

cudnn下载链接：https://developer.nvidia.com/cudnn

tensorRT下载链接：https://developer.nvidia.com/nvidia-tensorrt-8x-download

ref:https://github.com/countsp/ubuntu_settings/blob/main/nvidia.md

下载cuda安装的前置包：

```shell
$ sudo apt-get install freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev
```

随后根据下载链接中的引导完成安装

这里我安装了CUDA12.1 cudnn8.9.7 TensorRT 8.6 GA

```shell
tar -zxvf TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0.tar.gz
```

add to bashrc

```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/TensorRT-8.6.1.6/lib
```

​    

为了避免其它软件找不到 TensorRT 的库，建议把 TensorRT 的库和头文件添加到系统路径下

```shell
    # TensorRT路径下
    sudo cp -r lib/* /usr/lib
    sudo cp -r include/* /usr/include
```



## 安装ROS 2 Galactic

```shell
wget http://fishros.com/install -O fishros && . fishros
```

只需这一行命令，跟随指示选择即可，如果有基础版和桌面版的选择，选择桌面版



## 安装RMW Implementation

```shell
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt update
sudo apt install ros-galactic-rmw-cyclonedds-cpp


# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

如果使用fishros安装的话，会提示以及安装过了



## 安装pacmod

```shell
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-galactic-pacmod3
```



## 安装 Autoware Core dependencies

```shell
sudo apt install python3-pip
pip3 install gdown
```



## 安装Autoware Universe dependencies

```shell
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm2008-1
```



## 安装pre-commit dependencies

```shell
pip3 install pre-commit clang-format==17.0.6
sudo apt install golang
```



## Set up a workspace

进入autoware目录下

```shell
cd autoware
vcs import src < autoware.repos
vcs pull src
```

***报错提示***

*<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730104310773.png" alt="image-20240730104310773" style="zoom: 67%;" />*

*解决方法：*

```shell
sudo apt-get install python3-vcstool
```

*出了新的错*

*![image-20240730112923577](/home/bydwyf/.config/Typora/typora-user-images/image-20240730112923577.png)*

*只需在autoware文件夹下创建src文件夹即可*

*<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730125631756.png" alt="image-20240730125631756" style="zoom:50%;" />*

*vcs pull时会提示不在某个分支上，暂时未解决*<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730130036308.png" alt="image-20240730130036308" style="zoom:50%;" />



继续向下进行

Install dependent ROS packages.

```shell
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

*报错提示：*

*按照提示进行操作*

![image-20240730130153053](/home/bydwyf/.config/Typora/typora-user-images/image-20240730130153053.png)

```shell
sudo rosdep init
rosdep update
```

*在进行 rosdep init 时报错：*

<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730130611702.png" alt="image-20240730130611702" style="zoom: 50%;" />

try this:

```shell
sudo -E rosdep init
rosdep update
```

*time out了，决定换国内源*

```shell
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update
```

*初始化完成*

*再进行上面操作时，再次报错：*

```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
lanelet2_map_preprocessor: Cannot locate rosdep definition for [pcl_ros]
tier4_autoware_api_launch: Cannot locate rosdep definition for [topic_tools]
lidar_apollo_segmentation_tvm_nodes: Cannot locate rosdep definition for [ros_testing]
autoware_point_types: Cannot locate rosdep definition for [point_cloud_msg_wrapper]
lidar_apollo_segmentation_tvm: Cannot locate rosdep definition for [tvm_vendor]
grid_map_octomap: Cannot locate rosdep definition for [octomap]
autoware_testing: Cannot locate rosdep definition for [ros_testing]
external_velocity_limit_selector: Cannot locate rosdep definition for [topic_tools]
accel_brake_map_calibrator: Cannot locate rosdep definition for [diagnostic_updater]
sample_sensor_kit_launch: Cannot locate rosdep definition for [usb_cam]
obstacle_collision_checker: Cannot locate rosdep definition for [pcl_ros]
tvm_utility: Cannot locate rosdep definition for [tvm_vendor]
tier4_localization_launch: Cannot locate rosdep definition for [topic_tools]
steer_offset_estimator: Cannot locate rosdep definition for [diagnostic_updater]
tier4_vehicle_launch: Cannot locate rosdep definition for [xacro]
system_monitor: Cannot locate rosdep definition for [diagnostic_updater]
sample_vehicle_description: Cannot locate rosdep definition for [xacro]
web_controller: Cannot locate rosdep definition for [rosbridge_server]
grid_map_costmap_2d: Cannot locate rosdep definition for [nav2_costmap_2d]
radar_tracks_msgs_converter: Cannot locate rosdep definition for [radar_msgs]
image_projection_based_fusion: Cannot locate rosdep definition for [pcl_ros]
probabilistic_occupancy_grid_map: Cannot locate rosdep definition for [pcl_ros]
bluetooth_monitor: Cannot locate rosdep definition for [diagnostic_updater]
osqp_interface: Cannot locate rosdep definition for [osqp_vendor]
velodyne_description: Cannot locate rosdep definition for [xacro]
fault_injection: Cannot locate rosdep definition for [diagnostic_updater]
grid_map_cv: Cannot locate rosdep definition for [filters]
lane_departure_checker: Cannot locate rosdep definition for [diagnostic_updater]
topic_state_monitor: Cannot locate rosdep definition for [diagnostic_updater]
pointcloud_preprocessor: Cannot locate rosdep definition for [point_cloud_msg_wrapper]
front_vehicle_velocity_estimator: Cannot locate rosdep definition for [pcl_ros]
grid_map_demos: Cannot locate rosdep definition for [octomap_msgs]
lidar_centerpoint_tvm: Cannot locate rosdep definition for [tvm_vendor]
velodyne_pointcloud: Cannot locate rosdep definition for [pcl_ros]
autoware_auto_geometry: Cannot locate rosdep definition for [osrf_testing_tools_cpp]
trtexec_vendor: Cannot locate rosdep definition for [tensorrt_cmake_module]
lanelet2_extension: Cannot locate rosdep definition for [lanelet2_validation]
map_loader: Cannot locate rosdep definition for [pcl_ros]
system_error_monitor: Cannot locate rosdep definition for [rqt_robot_monitor]
lidar_centerpoint: Cannot locate rosdep definition for [pcl_ros]
localization_error_monitor: Cannot locate rosdep definition for [diagnostic_updater]
occupancy_grid_map_outlier_filter: Cannot locate rosdep definition for [pcl_ros]
velodyne_driver: Cannot locate rosdep definition for [diagnostic_updater]
planning_error_monitor: Cannot locate rosdep definition for [diagnostic_updater]
pacmod_interface: Cannot locate rosdep definition for [diagnostic_updater]
obstacle_stop_planner: Cannot locate rosdep definition for [pcl_ros]
tensorrt_common: Cannot locate rosdep definition for [tensorrt_cmake_module]
scenario_selector: Cannot locate rosdep definition for [topic_tools]
grid_map_filters: Cannot locate rosdep definition for [filters]
dummy_diag_publisher: Cannot locate rosdep definition for [diagnostic_updater]
vehicle_cmd_gate: Cannot locate rosdep definition for [diagnostic_updater]
velodyne_monitor: Cannot locate rosdep definition for [diagnostic_updater]
tier4_api_msgs: Cannot locate rosdep definition for [geographic_msgs]
image_diagnostics: Cannot locate rosdep definition for [diagnostic_updater]
external_cmd_converter: Cannot locate rosdep definition for [diagnostic_updater]
grid_map_ros: Cannot locate rosdep definition for [nav2_msgs]
behavior_path_planner: Cannot locate rosdep definition for [behaviortree_cpp_v3]
radar_threshold_filter: Cannot locate rosdep definition for [radar_msgs]
awsim_sensor_kit_launch: Cannot locate rosdep definition for [usb_cam]
trajectory_follower: Cannot locate rosdep definition for [diagnostic_updater]
costmap_generator: Cannot locate rosdep definition for [pcl_ros]
awapi_awiv_adapter: Cannot locate rosdep definition for [topic_tools]
external_cmd_selector: Cannot locate rosdep definition for [diagnostic_updater]
obstacle_velocity_limiter: Cannot locate rosdep definition for [pcl_ros]
radar_static_pointcloud_filter: Cannot locate rosdep definition for [radar_msgs]
ground_segmentation: Cannot locate rosdep definition for [pcl_ros]
tensorrt_yolox: Cannot locate rosdep definition for [tensorrt_cmake_module]
```

*有如下这些包，用这段命令安装：*

```shell
sudo apt-get install -y ros-galactic-pcl-ros \
ros-galactic-radar-msgs \
ros-galactic-topic-tools \
ros-galactic-diagnostic-updater \
ros-galactic-ros-testing \
ros-galactic-behaviortree-cpp-v3 \
ros-galactic-usb-cam \
ros-galactic-nav2-msgs \
ros-galactic-lanelet2-validation \
ros-galactic-octomap \
ros-galactic-tvm-vendor \
ros-galactic-xacro \
ros-galactic-point-cloud-msg-wrapper \
ros-galactic-nav2-costmap-2d \
ros-galactic-filters \
ros-galactic-tensorrt-cmake-module \
ros-galactic-osqp-vendor \
ros-galactic-osrf-testing-tools-cpp \
ros-galactic-octomap-msgs \
ros-galactic-rosbridge-server \
ros-galactic-geographic-msgs \
ros-galactic-rqt-robot-monitor \
ros-galactic-lanelet2-maps \
ros-galactic-diagnostic-aggregator \
ros-galactic-rqt-runtime-monitor \
ros-galactic-cudnn-cmake-module \
ros-galactic-ament-clang-format \
librange-v3-dev \
libpcl-dev \
libfmt-dev \
libcpprest-dev \
libpcap-dev \
libcgal-dev \
nlohmann-json3-dev \
ros-galactic-ublox-gps \
ros-galactic-can-msgs \
ros-galactic-pacmod3-msgs \
libopenni-dev \
libopenni2-dev \
libpcap-dev \
libpng-dev \
libusb-1.0-0-dev \
libyaml-cpp-dev \
pkg-config \
libnl-genl-3-dev
```

再次进行install

```shell
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730140127907.png" alt="image-20240730140127907" style="zoom:50%;" />

osrf_testing_tools_cpp报错暂不需处理



创建 workspace

```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

*报错*

<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730141153105.png" alt="image-20240730141153105" style="zoom: 67%;" />

*针对 tensorrt_common的报错：*

找到了/work/autoware/src/universe/autoware.universe/common/tensorrt_common这个路径下tensorrt_common 的CMakeList.txt最后一行加入

```txt
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-parameter")
```

再次运行

*报错*

<img src="/home/bydwyf/.config/Typora/typora-user-images/image-20240730160347051.png" alt="image-20240730160347051" style="zoom: 67%;" />

这次是![image-20240730160400994](/home/bydwyf/.config/Typora/typora-user-images/image-20240730160400994.png)

太多出错，认为是tensorrt的版本问题，决定重新更换版本
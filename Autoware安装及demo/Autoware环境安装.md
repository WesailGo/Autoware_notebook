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

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240729151328452.png" alt="image-20240729151328452" style="zoom:50%;" />

找到recommended，接下来去 软件与更新---->附加驱动 选择相应的驱动，点击应用更改，重启电脑即可，无需禁用原驱动（会导致黑屏）

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240729151515701.png" alt="image-20240729151515701" style="zoom:50%;" />



## 克隆Autoware仓库

```shell
git clone https://github.com/autowarefoundation/autoware.git -b galactic
cd autoware
```

## 安装CUDA、Cudnn、TensorRT

cuda 下载链接：https://developer.nvidia.com/cuda-toolkit-archive

cudnn下载链接：https://developer.nvidia.com/cudnn

tensorRT下载链接：https://developer.nvidia.com/nvidia-tensorrt-8x-download

ref:https://github.com/countsp/ubuntu_settings/blob/main/nvidia.md

~~下载cuda安装的前置包：~~

~~随后根据下载链接中的引导完成安装~~

~~这里我安装了CUDA12.1 cudnn8.9.7 TensorRT 8.6 GA~~

~~为了避免其它软件找不到 TensorRT 的库，建议把 TensorRT 的库和头文件添加到系统路径下~~

需重新安装cuda 11.6.0、cudnn 8.4.0.27、tensorrt 8.4.2.4

首先卸载原cuda、cudnn、tensorrt

删除tensorrt：删除掉之前解压的文件夹，移除环境变量

卸载cuda：

```shell
cd /usr/local/cuda-12.1/bin
sudo ./cuda-uninstaller
回车选中这三个然后选中done
sudo rm -rf /usr/local/cuda-12.1
```

卸载cudnn：删除原来的cuda即可，可以把原来的包也删除

### **接下来重新安装**

#### 安装CUDA11.6.0 

https://developer.nvidia.com/cuda-11-6-0-download-archive

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240731105540564.png" alt="image-20240731105540564" style="zoom:50%;" />

```shell
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.6.0/local_installers/cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-6-local/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda
```

添加环境变量：

```shell
export PATH=$PATH:/usr/local/cuda/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
export CUDA_HOME=$CUDA_HOME:/usr/local/cuda

cd /usr/local
sudo ln -s ./cuda-11.6/ ./cuda         (软链接换成安装的cuda版本)
```

#### cudnn安装

https://developer.nvidia.com/rdp/cudnn-archive

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240731110735729.png" alt="image-20240731110735729" style="zoom:67%;" />

解压后：

```shell
sudo cp cudnn-linux-x86_64-8.4.0.27_cuda11.6-archive/include/* /usr/local/cuda-11.6/include
sudo cp cudnn-linux-x86_64-8.4.0.27_cuda11.6-archive/lib/libcudnn* /usr/local/cuda-11.6/lib64
sudo chmod a+r /usr/local/cuda-11.6/include/cudnn.h
sudo chmod a+r /usr/local/cuda-11.6/lib64/libcudnn*
```

```shell
cat /usr/local/cuda-11.6/include/cudnn_version.h | grep CUDNN_MAJOR -A 2  
#使用该命令进行验证
```

#### TensorRT安装

https://developer.nvidia.com/nvidia-tensorrt-8x-download

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240731140215101.png" alt="image-20240731140215101" style="zoom:50%;" />

```shell
tar -zxvf TensorRT-8.4.2.4.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/bydwyf/work/TensorRT-8.4.2.4/lib

# TensorRT路径下
sudo cp -r ./lib/* /usr/lib
sudo cp -r ./include/* /usr/include

# 安装TensorRT
cd TensorRT-8.4.2.4/python
pip install tensorrt-8.4.2.4-cp38-none-linux_x86_64.whl
 
# 安装UFF,支持tensorflow模型转化
cd TensorRT-8.4.2.4/uff
pip install uff-0.6.9-py2.py3-none-any.whl
 
# 安装graphsurgeon，支持自定义结构
cd TensorRT-8.4.2.4/graphsurgeon
pip install graphsurgeon-0.4.6-py2.py3-none-any.whl
```

然后验证安装是否成功，进入到TensorRT-8.5.3.1/samples/sampleOnnxMNIST路径下，执行

```
sudo make    
```

编译成功后显示可执行那个文件在如下目录(TensorRT-8.5.3.1/bin)

```
./sample_onnx_mnist
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
mkdir src
vcs import src < autoware.repos
vcs pull src
```

***报错提示***

*<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730104310773.png" alt="image-20240730104310773" style="zoom: 67%;" />*

*解决方法：*

```shell
sudo apt-get install python3-vcstool
```

继续向下进行

Install dependent ROS packages.

```shell
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

*报错提示：*

*按照提示进行操作*

![image-20240730130153053](/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730130153053.png)

```shell
sudo rosdep init
rosdep update
```

*在进行 rosdep init 时报错：*

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730130611702.png" alt="image-20240730130611702" style="zoom: 50%;" />

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

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730140127907.png" alt="image-20240730140127907" style="zoom:50%;" />

osrf_testing_tools_cpp报错暂不需处理



创建 workspace

```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

*报错*

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730141153105.png" alt="image-20240730141153105"  />

*针对 tensorrt_common的报错：*

找到了/work/autoware/src/universe/autoware.universe/common/tensorrt_common这个路径下tensorrt_common 的CMakeList.txt最后一行加入

```txt
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-parameter")
```

再次运行

*报错*

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730160347051.png" alt="image-20240730160347051"  />

这次是![image-20240730160400994](/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240730160400994.png)

一直报错是 error: ‘nvinfer1::Dims nvinfer1::ICudaEngine::getBindingDimensions(int32_t) const’ is deprecated [-Werror=deprecated-declarations]

处理方法----->重新安装CUDA11.6.0、cudnn8.4.0.27、tensorRT 8.4.2.4 （已更新）

*更新后报错：*

<img src="/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240731143542407.png" alt="image-20240731143542407" style="zoom:50%;" />

进入tensorrt-populate-gitclone.cmake修改里面的8.6.1为8.4.2，因为之前的tensorrt是8.6.1的版本

```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

一直不成功，把src文件夹删掉，又从回收站恢复，再次运行，**成功！**（太玄学了）

![image-20240731165121480](/home/bydwyf/work/Autoware_notebook/Autoware安装及demo/Autoware环境安装.assets/image-20240731165121480.png)

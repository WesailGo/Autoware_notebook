# Autoware环境安装

> Ref: 
>
> - https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/
> - https://blog.csdn.net/zardforever123/article/details/132029636

### 环境配置：

- Ubuntu20.04
- Nvidia A4000

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

```
$ sudo apt-get install freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev
```

随后根据下载链接中的引导完成安装

这里我安装了CUDA12.1 cudnn8.9.7 TensorRT 8.6 GA

```
tar -zxvf TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0.tar.gz
```

add to bashrc

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/TensorRT-8.6.1.6/lib
```

​    

为了避免其它软件找不到 TensorRT 的库，建议把 TensorRT 的库和头文件添加到系统路径下

```
    # TensorRT路径下
    sudo cp -r lib/* /usr/lib
    sudo cp -r include/* /usr/include
```



## 安装ROS 2 Galactic

```
$ wget http://fishros.com/install -O fishros && . fishros
```

只需这一行命令，跟随指示选择即可，如果有基础版和桌面版的选择，选择桌面版



## 安装ros2_dev_tools

```shell
# Taken from https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html
$ sudo apt update && sudo apt install -y \
  python3-colcon-mixin \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures

# Initialize rosdep
$ sudo rosdep init
$ rosdep update
```

这个方法不行，显示没有公钥
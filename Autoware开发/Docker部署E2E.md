# Docker部署E2E

### 安装Docker

安装必要依赖

```shell
sudo apt update
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent
```

设置源

```shell
1. 首先设置密钥
# 官方密钥（不建议）
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# 阿里密钥（推荐）
curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
2. 设置软件源
# 官方源（不建议用）
echo \
"deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 阿里源（推荐）
echo \
"deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://mirrors.aliyun.com/docker-ce/linux/ubuntu \
$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

```

安装docker

```shell
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

验证

```
sudo docker run hello-world
```

如果pull出了超时问题，使用鱼香ros

```
wget http://fishros.com/install -O fishros && bash fishros 

选17

选2

选1
```



### 创建Docker容器并进行快照保存为镜像

##### 运行容器

```
sudo docker run -it ubuntu:20.04 /bin/bash
```

在容器内安装相关环境

> ref：https://github.com/qintonguav/ParkingE2E

```
git clone https://github.com/qintonguav/ParkingE2E.git
```

#### ROS Noetic

```shell
wget http://fishros.com/install -O fishros && . fishros
```

跟随指示安装

#### OpenCV 4

```shell
pip install opencv-python
```



安装好后，使用
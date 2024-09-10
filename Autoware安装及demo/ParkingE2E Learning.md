# ParkingE2E Learning

> 端到端泊车
>
> github：https://github.com/qintonguav/ParkingE2E
>
> paper：https://arxiv.org/pdf/2408.02061



## 安装环境：

- CUDA
- ROS Noetic
- Ubuntu 20.04
- OpenCV 4

Ubuntu20.04与CUDA参考 ./Autoware环境安装.md

#### CUDA

```
https://blog.csdn.net/weixin_43702653/article/details/129249585
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

按照github指示流程进行

其中运行下列命令时会下载 [efficientnet-b4-6ed6700e.pth](efficientnet-b4-6ed6700e.pth) ，下载会很慢，自己下载或取用后放置在终端中显示的下载后放置路径

```
python ros_inference.py
```



scene 1:

![image-20240815095109121](./ParkingE2E%20Learning_imgs/image-20240815095109121.png)

scene 2:

![image-20240815095322785](./ParkingE2E%20Learning_imgs/image-20240815095322785.png)

scene 3:

![image-20240815095448631](./ParkingE2E%20Learning_imgs/image-20240815095448631.png)

scene 4:

![image-20240815100104261](./ParkingE2E%20Learning_imgs/image-20240815100104261.png)

scene 5:

![image-20240815100412665](./ParkingE2E%20Learning_imgs/image-20240815100412665.png)

scene 6:

![image-20240815100437330](./ParkingE2E%20Learning_imgs/image-20240815100437330.png)

scene 7:

![image-20240815100523362](./ParkingE2E%20Learning_imgs/image-20240815100523362.png)



整理的网络结构如下图

![E2Enode.drawio](./ParkingE2E%20Learning_imgs/E2Enode.drawio.svg)

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



## 安装Ros2.galactic
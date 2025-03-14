# Elastic-Tracker部署指南

本文档是部署Elastic-Tracker到无人机上并进行仿真与实际验证的指南。  
说明文档由 张子宇 撰写。

## 简 介
[Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) 是一个灵活的轨迹规划框架，能够以保证安全和可见性的方式处理具有挑战性的跟踪任务。  
[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) 是一种基于优化的多传感器状态估计器，实现了无人机的精确自定位。

部署本仓库，你可以完成一架具有**稳定飞行与悬停**（由VINS实现）、**自主跟随目标并避障飞行**（由Elastic-Tracker实现）的无人机，或是在Gazebo中完成一架与实机部署流程基本一致的仿真飞机。

本仓库集成了定位算法VINS-Fusion、追踪算法Elastic-Tracker、硬件驱动Realsense2_camera，将它们整合在一个项目（工作空间）中，以实现部署即用的效果，简化大家的调试流程、减轻初学者的学习负担。
>原Elastic-Tracker项目并未开源目标识别算法。作者自写了一个较为鲁棒的（应该吧）目标识别算法，自动识别RGBD相机视野中的红色物体。你也可以针对本部分进行二次开发。

本项目在Gazebo中联合调试仿真通过并完成了所有功能：
>操作系统：Ubuntu 20.04  
>CPU：R9-7945HX  
>GPU：RTX-4060 Laptop 8G  
>内存：16G

本项目在以下实机平台上部署通过并完成了所有功能：
>机型：250mm轴距碳板机架 四旋翼  
飞控：雷迅V5+ / PX4固件  
机载计算机：Intel NUC i7-1165G7 / Ubuntu 20.04  
动力套：T-Motor V2207 V2.0 KV2550 ×4  
双目相机：Intel Realsense D435i  

## 仿真部署指南

使用本文档进行配置，需要**掌握基本的Ubuntu命令使用方法**，**了解ROS的基本概念与工作方式**。

>1. 了解Ubuntu中终端的意义与用法；
>2. 了解常见的bash语法；
>3. 了解环境变量的含义，会配置环境变量；
>4. 具备基本的C++编程能力；
>5. 了解ROS的工作空间、功能包、话题机制的运作原理；
>6. 了解PX4飞控的基本机制。

具备以上基础，才可能顺利的进行此项目的配置。

在Gazebo中对整个无人机进行仿真。为了更好的仿真效果，需要加载仿真场景，这对电脑的显卡配置要求较高，且需要针对Ubuntu和Gazebo配置显卡驱动与OpenGL相关配置。  
配置方法记录在文末的**环境配置说明**中。

### 1. 配置PX4固件、ROS、MAVROS

配置PX4可参照[PX4从放弃到精通（二）：ubuntu18.04配置px4编译环境及mavros环境](https://blog.csdn.net/qq_38768959/article/details/106041494?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167361309116782425683823%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fblog.%2522%257D&request_id=167361309116782425683823&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~blog~first_rank_ecpm_v1~rank_v31_ecpm-4-106041494-null-null.blog_rank_default&utm_term=gazebo&spm=1018.2226.3001.4450)一文配置PX4、ROS和MAVROS。  

>PX4配置过程中可能会报无法安装gazebo9（因为依赖冲突）的错误，这是正常现象，因为ubuntu20.04默认使用gazebo11。如果下一步的`make`指令能够正常运行，则可无视此问题。

>不推荐按照上面这篇教程安装ROS，非常复杂。请参照[这篇文章](https://blog.csdn.net/u011391361/article/details/131582593)的教程一键安装。不得安装ROS2，架构不互通。

本项目在PX4 v1.11.3固件上仿真通过，在v1.15.0固件上仿真和实机通过。鉴于部分新硬件无法被旧版固件支持，在确保稳定的前提下可用较高版本的固件。
>**注意！**  
按照PX4 v1.14.0版本固件的发行说明，自v1.14版本起，PX4的官方仿真环境将变更为新版Gazebo（只能在Ubuntu22.04及以上版本运行），而ubuntu20.04版本搭载的Gazebo（称为Gazebo Classic）仍然保留，但**变更了功能包所在路径**，因此使用高版本固件进行仿真时需要密切注意环境变量的设置。  
>
>目前大多数教程在配置环境变量时，都是：
>```bash
>source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
>export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
>```
>对于高版本固件来说，以上路径是不正确的。应当为（在v1.15.0下验证通过）：
>```bash
>source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
>export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
>```  
>请特别留意`~/.bashrc`中的环境变量设置！否则可能导致Gazebo无法正确加载出仿真环境和模型的问题。

如正确配置，则在PX4_Autopilot文件夹下：
```bash
make px4_sitl gazebo
```
可见gazebo正常启动，模型正常加载，表明配置完成。

### 2. 加载仿真环境与插件

为了仿真效果更好，这里加载仿真世界场景。gazebo默认没有安装加载世界场景的模型，因此需要自行安装。本项目已经收集、配置了许多模型与世界环境文件，通过如下命令下载：
```bash
git clone https://github.com/npu-ius-lab/Gazebo-Models.git
```

**注意！**默认下载应用于较新版本的PX4固件的分支。如果你打算使用旧版本PX4固件，请下载旧版本分支。  
所有资源大概2 GB左右。下载完成后，按照该仓库的Readme文件配置即可。

### 3. 启动Gazebo仿真

启动终端，执行：
```bash
roslaunch px4 mavros_posix_sitl.launch
```
稍等片刻后Gazebo应该正常启动，并加载出了仿真模型。（启动时经常报一堆红字，无所谓，能启动就是正常的）
![](/image//2025-02-12%2016-39-57%20的屏幕截图.png)

可以再打开一个终端，调用`rostopic list`，可以看到有许多和`/camera`有关的话题，说明双目相机模型、下视相机模型都启动成功。

最后启动QGC地面站，确保飞控连接。现在可以在QGC上在自稳模式下进行起飞了。记得在`Application Settings`中打开“虚拟游戏手柄”以操纵飞机。
![](/image/2025-02-12%2016-46-58%20的屏幕截图.png)
<font color=#696969>使用gazebo仿真时可以连接游戏手柄进行操纵。</font>

为确保视觉里程计正常运行，还需要配置一些参数。可以在QGC中更改或者在固件中更改，具体方法可查看[官方文档](https://docs.px4.io/v1.12/zh/advanced_config/parameters.html)。

在`v1.11.3`固件中，需要更改的参数为：
```
[安全检查]
CBRK_USB_CHK = 197848       //插USB时可解锁
CBRK_IO_SAFETY = 22027      //无安全开关可解锁
CBRK_SUPPLY_CHK = 894281    //无电流计可解锁

[定位数据]
EKF2_AID_MASK = 24          //定位数据来自视觉
EKF2_HGT_MODE = 3           //高度数据主要来源为视觉
```
在`v1.15.0`固件中，需要更改的参数为：
```
[安全检查]
CBRK_USB_CHK = 197848       //插USB时可解锁
CBRK_IO_SAFETY = 22027      //无安全开关可解锁
CBRK_SUPPLY_CHK = 894281    //无电流计可解锁

[定位数据]
EKF2_GPS_CTRL = 0           //禁用GPS数据融合
EKF2_EV_CTRL = 15           //启用视觉数据融合
EKF2_HGT_REF = 3            //高度数据主要来源为视觉
```
>按理来说，PX4采用的EKF2融合估计能够同时融合GPS和视觉里程计的位姿数据。但是，由于相机位姿需要从无人机位姿变换，如果在PX4的EKF2结果上处理会比较麻烦。所以禁用了PX4融合GPS数据。  

此外，还需要提高IMU的发布频率到200Hz，这不能在QGC的参数界面更改，而是需要从固件中更改或者从MAVLink控制台更改。可参阅[这篇教程](https://blog.csdn.net/qq_44998513/article/details/132683878)进行更改：
```
[发布频率（从固件更改）]
configure_stream_local("HIGHRES_IMU", 200.0f);
configure_stream_local("ATTITUDE_QUATERNION", 200.0f);
[发布频率（从控制台更改）]
mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 100
mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 100
```

### 4. 启动VINS_Fusion

首先需要配置VINS所需的依赖功能包。解压本项目文件夹下的`3rd_party.zip`，然后：

1. 进入`glog`文件夹，在文件夹中打开终端，执行：
   ```bash
   ./autogen.sh && ./configure && make && sudo make install
   ```
   >若遇到权限问题，在文件夹中找到`autogen.sh`和`configure`文件，右击属性，在权限中选择“允许文件作为程序执行”。
   
   然后安装下面的库：
   ```bash
   sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
   ```

2. 进入`ceres-solver-2.0.0rc1`文件夹，在文件夹中打开终端，按顺序执行：
   ```bash
   mkdir build
   cd build
   cmake ..
   sudo make -j4
   sudo make install
   sudo apt-get install ros-noetic-ddynamic-reconfigure
   ```
这样就完成了必要的依赖安装。安装完成后你可以把原始文件删除。
>注意，VINS还依赖openCV库，它在ubuntu20.04中已经预装，但为openCV4版本。原生的VINS是基于ubuntu18.04开发的，因此不通用。  
如果你要使用ubuntu18.04，请自行前往该项目的github克隆。

然后检查VINS的外参输出的文件夹路径是否合适。打开`VINS-Fusion/config/gazebo/Drone_gazebo.yaml`文件，检查`output_path`的值与VINS下该文件夹的真实路径是否吻合（一般是用户名需要修改）。

接下来在本项目的文件夹（即工作空间中）启动终端，进行编译：
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```

然后，依次执行以下命令：
```bash
source devel/setup.bash
roslaunch vins Drone_gazebo.launch
```
这会应用当前项目的环境变量并加载VINS，并在Rviz观察位姿的可视化数据。

>`source devel/setup.bash`是一句常见的引用环境变量的语句，如果不想每次这么麻烦，可以使用启动脚本。详见本部分的末尾介绍。

>请注意检查相机的参数。如克隆配置好的文件则无需修改。  
否则，应当检查`/VINS-Fusion/config/gazebo`文件夹下的`left_gazebo.yaml`和`right_gazebo.yaml`参数文件。

>调用`rostopic echo /camera/infra1/camera_info`即可检查左目的参数。  
除0以外，K矩阵中应该有两个相等的值（第一个和第三个）分别是fx,fy，第二个是cx，第四个则是cy。  
将`infra1`的1换成2即为右目，参数同理。

此时应该可以在QGC - Analyze Tools - MAVLink检测中看到`LOCAL_POSITION_NED`数据。Rviz中也可以看见双目数据了。
![](/image/2025-02-12%2016-50-37%20的屏幕截图.png)

通过`rostopic hz /mavros/imu/data_raw`检查imu发布频率，应当在200Hz左右或以上方为正常，至少要大于100Hz。过低的IMU频率会导致VINS漂移。
>Gazebo中的Realsense相机仿真不太稳定，经常出现`throw img0`的警告信息。这无妨，有时VINS会飘，飘了重启就好了。

>**参数标定**（仿真时非必须）：  
能够起飞后，用`rostopic echo /vins_fusion/imu_propagate`检查里程计话题，同时在地面站中小幅度飞行飞机，然后降落。  
此时`/output/gazebo`文件夹中应该产生了`extrinsic_parameter.txt`文件，将其中的参数矩阵复制到`/config/gazebo/Drone_gazebo.yaml`文件中的相同位置，即完成外参标定。

在地面站中将飞行模式切换为Position，应当能够切换并顺利起飞、悬停.

### 5. 检查OpenCV-Target

本项目使用opencv做物体识别，将RGB相机获得的图像信息进行掩模处理后筛选出红色色块，再从对齐后的深度图中获得目标色块的深度信息，结合相机位姿就能获得物体在世界坐标系下的位置。
>因此深度相机是必须项，没有深度值就不能做到避障和追踪。

>如果要扩展本项目，例如想要识别人像并追踪，也只需要更改这部分的功能即可。接口信息详见*代码框架介绍*。

在工作空间下新启动一个终端，执行以下命令：
```bash
source devel/setup.bash
roslaunch opencv_target test_gazebo.launch
```
这会启动两个节点：基于opencv的识别程序opencv_target，和使用ekf滤波估计目标位姿的程序target_ekf。

现在，你可以新启动一个终端，输入`rqt_image_view`，应当可以看到有一个`color_image_processed`话题输出。如果有红色物体出现在画面中，则该话题的图像会输出一个框将其框出。
>注意，颜色识别与物体的光照、明暗等亦有关系。Gazebo中也会仿真光照条件，因此如果识别不到物体，在确认颜色正确的情况下，尝试更换一个光照良好的角度。

### 6. 启动Elastic-Tracker

打开`Target-Tracking-Drone-250/src/Elastic-Tracker/src/planning/planning/launch/run_in_gazebo.launch`文件，检查：
- 节点`opencv_target_node`下的深度与RGB图像话题。
- 节点`target_ekf_node`下的相机位姿话题。
- 节点`nodelet... arg="load mapping/Nodelet manager"`中加载的相机参数。详见“参数标定”。
- 节点`nodelet... arg="load planning/Nodelet manager"`最大飞行速度和加速度。不要太快即可。
>如果对ROS不熟悉，不要轻易更改话题名称。

确认无误后，在工作空间下新打开一个终端，执行：
``` bash
source devel/setup.bash
roslaunch planning run_in_gazebo.launch
```

现在，从Position模式将飞机起飞，悬停稳定后再切入Offboard模式，如果一切正常，飞机现在应当可以在Offboard模式下悬停。

如果在`rqt_image_view`窗口中可以看见被跟踪的目标，则无人机应当能够跟踪目标。你可以在gazebo中拖动红色的小方块，看无人机如何运动。本项目的`image`文件夹中给出了一个示例。

### 7. 联合启动

如果以上调试均无问题，你可以在本项目的目录下启动终端，输入：
```bash
./run_in_gazebo.sh
```
所有功能会在一个终端的三个选项卡下依次启动。

## 实机部署指南

除更改话题订阅名称外，基本流程一致。

#### 1. 硬件部署

在部署机载计算机以前，首先需要一台能够在自稳模式下正常起飞的无人机。这需要部署动力套，包括电机、电调、分电板、电流计、飞控、接收机等。本项目的重点并不是部署无人机，因此不开展这部分内容的介绍。

若无人机已部署完毕，则应当完成以下接线（以雷迅V5+飞控为例）：  
1. 飞控 <-> 机载电脑：通过USB-TypeC线连接
2. D435i <-> 机载电脑：通过USB-TypeC线连接
>当然你也可以用串口线连接PX4飞控的TELEM1和电脑，只是需要一个USB转串口模块，并且配置方法（主要是端口名称）略有不同。

#### 2. 通讯连接

首先进行MAVROS的配置，这是重中之重，如果没有MAVROS机载计算机将无法与PX4通信。配置方法与仿真环节基本一致。如果你已经安装好了ROS和MAVROS，则在机载电脑上启动终端：
```bash
roslaunch mavros px4.launch
```
应当能够见到它正常启动，不报错（但这个时候应该还没有连接上PX4）。下面来配置MAVROS与PX4的连接端口。

首先打开一个终端，执行：
```bash
ls /dev
```
你会看见很多名称，找到其中名为`ttyACM0`（或ACM1等，可能有几个），记下来。然后拔掉PX4与机载电脑的连线，再执行以上命令，看看哪个名称消失了，说明这个就是**PX4与机载电脑连接的端口名称**，**记住它**。

打开`/opt/ros/noetic/share/mavros/launch`文件夹，执行以下命令：
```bash
sudo gedit px4.launch
```
在打开的文本编辑器里更改以下两行参数的值为：
```xml
<arg name="fcu_url" default="/dev/ttyACM0:921600" />
<arg name="gcs_url" default="udp://@localhost" />
```
其中的`ttyACM0`更改为你自己的**端口名称**。

打开一个终端，运行以下命令为串口赋权限：
```bash
sudo chmod +x /dev/ttyACM0
```
其中的`ttyACM0`同样更改为你自己的**端口名称**。

然后再次执行：
```bash
roslaunch mavros px4.launch
```
这时候MAVROS就应该正常启动了，`rostopic list`应该有话题输出。如果没有成功启动就再插拔一下USB线。

另一种查看是否启动的方式是使用QGC地面站。你可以参照[官方教程](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu)在你的机载电脑上安装。打开后点击左上角图标，在`Application Settings`中，调整下方的选项：
![](/image/2025-03-12%2017-35-40%20的屏幕截图.png)
>如果不这样设置，MAVROS会和QGC的通信链路发生冲突。  
细心观察可以发现，按照这套设置，MAVROS和PX4以ACM（虚拟串口）通信，而QGC与PX4以UDP协议通信，它们都通过USB线进行数据传输，协议均为虚拟实现的。

完成以上配置后，执行`roslaunch`，再打开QGC，应当能看见PX4与QGC正常连接了。

#### 3. Realsense驱动

仿真与实机的不同之一在于深度相机需要使用真实的Realsense相机，因此需要为其安装驱动。依次执行以下命令：

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
然后连接上D435i，在终端中启动`realsense-viewer`，可见深度相机正常显示。注意测试时左上角显示的USB必须是3.x，如果是2.x，可能是USB线是2.0的，或者插在了2.0的USB口上（3.0的线和口都是蓝色的）。

#### 4. 联合启动

如果确保仿真流程全部能够跑通，则将该项目的文件夹克隆到机载电脑上，在机载电脑上安装好所有依赖后，再在项目文件夹下启动如下执行`catkin_make`，编译通过表明代码能够正常运行。

在项目文件夹下执行：
```bash
./run_in_exp.sh
```
应当能够看见一个新终端启动，出现三个选项卡，分别启动了Realsense, VINS和Elastic Tracker。  
如果Realsense节点成功启动，应当见到`Realsense Node is UP!`；  
如果VINS节点成功启动，应当见到`Initialization finish!`；  
如果Elastic-Tracker节点成功启动，应当见到`PX4 is Not in offboard mode. Ready to hover.`持续发布。

现在，你可以按照正常起飞流程进行飞行。在此之前的飞控校准、遥控器通道设置等内容非常基础，不做讲解。

## 实现原理介绍

>所有代码均在Ubuntu20.04上的ROS Noetic运行。

整个项目以ROS为通信框架，通过VINS_Fusion进行位姿估计以得到无人机准确位置，通过OpenCV识别红色目标物体，再通过Elastic-Tracker进行追踪路径规划。

**1. 位姿估计**

主节点：`/vins_fusion`  

输入话题：  
`/mavros/imu/data_raw`  
`/camera/infra1/image_rect_raw`  
`/camera/infra2/image_rect_raw`  

输出话题：  
`/vins_fusion/imu_propagate`  
`/mavros/vision_pose/pose`  
`/vins_fusion/extrinsic`等  

该部分接收IMU、双目、GPS的消息，将其融合为里程计信息，分别发布给PX4和EGO-Planner。输入话题的来源在`config/Drone-250.yaml`中定义。

主要的输出话题有两个，一个是`/vins_fusion/imu_propagate`，用于发送给Elastic-Tracker；另一个是`/mavros/vision_pose/pose`，用于发送给PX4。两者同时发布，除了数据结构有所区别，在内容上没有任何差异。

此外，为了解算识别到的目标在世界坐标系下的位姿，还以`nav_msgs/Odometry`格式的消息发布了外参（相机到IMU的旋转与平移）。在求解识别物体在世界坐标系下的位姿时，需要此外参将坐标从相机坐标系转换到世界坐标系。

**2. 轨迹规划**

主节点：`/manager/mapping`,`/manager/planning`

输入话题：  
`/camera/depth/image_rect_raw`  
`/vins_fusion/extrinsic`  
`/vins_fusion/imu_propagate`  
`/target_ekf_node/target_odom`  
`/mavros/state`

输出话题：  
`/mavros/setpoint_raw/local`等

该部分接收里程计信息与深度相机信息，`mapping`模块由深度图建立局部栅格占据地图以进行避障，`planning`模块通过栅格地图和获得的目标位姿进行轨迹规划。最终由`traj_server`节点将位置、速度、加速度指令直接通过MAVROS发给PX4。

**3. 颜色识别**

主节点：`/opencv_traget_node`,`/target_ekf_node`

输入话题：  
`/camera/color/image_raw`  
`/camera/aligned_depth_to_color/image_raw`  
`/vins_fusion/imu_propagate`
`/vins_fusion/extrinsic`

输出话题：  
`/target/bbox`  
`/target_ekf_node/yolo_odom`
`/target_ekf_node/target_odom`

`/opencv_traget_node`为自编的程序部分，采用openCV库进行颜色识别，创建红色掩模提取识别红色区域的信息，并框选出第一个区域，利用RANSAC算法提取框内的像素坐标。通过对齐后的深度图获取目标像素点的深度，将目标从像素坐标系下的二维坐标转换到世界坐标系下的三维坐标。

发布的消息格式为yolo惯用的`boundingbox`类型消息，即包含了目标框的四个角点的像素坐标的信息。本项目还为目标框增加了深度信息，使用深度相机的数据对目标框内的物体进行位姿估计。

>因此，本部分的输入是像素坐标系下的二维坐标及其深度值。如果要更改识别目标，只需要获得目标的这个值即可。 

`/target_ekf_node`是用于对输入目标位置进行滤波的部分，通过EKF滤波器预测目标的位姿与速度，并发布在`target_odom`话题上以供`planning`模块调用。

**4. 控制逻辑**

采用雷迅CUAV V5+控制器与开源的PX4飞控固件，在Stablized自稳、Position定点、Offboard板外三种模式下控制。

**Stablized自稳：**  
遥控器完全接管，杆量为速度，不能悬停。  
此模式下无需任何设置。

**Position定点：**  
遥控器完全接管，杆量为位置增量，可以悬停。  
此模式下需要可靠的位姿信息，由VINS提供。VINS发布的位姿信息和PX4接受的位姿信息在数据上是一致的，只是封装的数据类型不同，在`visualization.cpp`中已经更改。
>注意话题映射，`vins`节点的句柄是私有于“\~”的，话题映射时前面加上“\~”。

**Offboard板外：**  
程序完全接管，遥控器不起作用。  
此模式下由Elastic-Tracker发送位置指令，由飞控自动执行。
>进入offboard模式前需要上位机程序持续发送位置指令给PX4以表明板外控制程序活跃。该部分逻辑在planning功能包中实现，通过订阅`/mavros/state`话题判断PX4的状态，不在offboard模式时持续发布当前位置作为位置指令。

## 程序框架介绍

在`src`（程序代码）文件夹下，存放了本项目的所有代码，主要分为以下三个文件夹：

识别、追踪程序：Elastic-Tracker
>**Elastic-Tracker**
>>`detection`  —— 识别目标并估计世界坐标  
>>`mapping` —— 建立栅格占据地图  
>>`planning`  —— 规划轨迹并发布   
>>`pose_utils`  —— 位姿公用处理  
>>`quadrotor_msgs`  —— 公用消息定义  

定位、建图程序：VINS-Fusion
>**VINS-Fusion**  
>>`camera_models` —— 建立相机模型  
>>`config` —— 存放参数文件  
>>`docker` —— 关于docker的文件  
>>`global_fusion` —— 松耦合GPS估计  
>>`loop_fusion` —— 回环检测  
>>`output` —— 外参标定输出  
>>`vins_estimator` —— 主程序 

Realsense驱动：Realsense2-Camera
>**Realsense2-Camera**  
>>`launch` —— 启动文件   
>>`msg` —— 消息类型  
>>`src` —— 主要代码  

>注：Realsense驱动可以不随你的工作空间部署，而是直接部署在机载计算机中。当然，不同项目对双目相机的参数（例如说帧率、分辨率等）可能不同，建议最好还是为每个项目单独配一个驱动功能包。

## 环境配置说明

为了流畅运行gazebo仿真，推荐使用带有独立显卡的电脑，且在安装nvidia显卡驱动时一并安装openGL。

**注意：**
[部分教程]((https://blog.csdn.net/lc250123/article/details/96428089))提到安装OpenGL会导致系统卡在启动循环，这是因为Ubuntu20.04自带的显示管理器为GDM导致的。请参考[这篇教程](https://blog.csdn.net/qq_19734597/article/details/103863892)将显示管理器**配置为lightDM**，再照下面的流程配置显卡驱动。

首先检查是否安装了显卡驱动。输入命令`nvidia-smi`，如果显示下图所示的界面，则说明显卡驱动已经成功安装。
![](/image/2025-02-12%2016-06-51%20的屏幕截图.png)

如果安装了显卡驱动，则启动gazebo。
如果可见`Processes`一栏中出现了`gzserver`和`gzclient`，说明gazebo已经被设置成显卡驱动的工作模式了，无需下面的设置。否则，需要按下面的流程为gazebo配置驱动。
![](/image/2025-02-12%2016-10-24%20的屏幕截图.png)

如已安装显卡驱动，在命令行界面卸载再次安装。一般都推荐使用run文件安装，因此也使用run文件卸载。

参照[这篇教程](https://blog.csdn.net/lc250123/article/details/96428089)的方法进行卸载：

先下载好显卡驱动的run文件，并将它放在home目录下，用`sudo chmod 777 ...`命令给文件赋予权限。（后面的`...`换成自己的文件名）

然后进入命令行界面：`sudo service lightdm stop`，执行前务必记住自己ubuntu的**用户名和密码**。

稍等片刻后按Ctrl+Alt+F1进入命令行界面，此时桌面会消失，不要慌张，片刻后会进入命令行界面。输入用户名和密码登录即可。注意这时小键盘是失效的。

然后执行卸载程序：`sudo ./NVIDIA-Linux-x86_64-384.59.run –-uninstall`。<font color=#696969>根据下载的显卡驱动不同，文件名有所不同，请使用Tab键进行补全。</font>

卸载后，依照[Gazebo贼卡，使用GPU加速重装显卡驱动无效解决方法](https://blog.csdn.net/weixin_63843256/article/details/145191913)和[Gazebo GPU加速【gzserver running in GPU】](https://blog.csdn.net/qq_38853759/article/details/132522471)教程再次安装。

**注意：**
安装时**不要**按照某些教程的指导**加上`–no-opengl-files`参数**！（但是，确保你已经将显示管理器切换到了**lightDM**）

安装完成后执行`sudo service lightdm start`即可恢复至图形界面。

使用`nvidia-smi`命令可查看驱动是否安装成功。

如安装成功，执行`inxi -G`（提示没有此命令则先安装：`sudo apt-get install inxi`），看见`OpenGL: renderer`后面显示自己的显卡型号则安装成功，如下图所示。
![](/image/2025-02-12%2016-16-18%20的屏幕截图.png)

再启动gazebo，并输入命令`nvidia-smi`，即可见gazebo用显卡驱动起来了，仿真帧率也显著上升。

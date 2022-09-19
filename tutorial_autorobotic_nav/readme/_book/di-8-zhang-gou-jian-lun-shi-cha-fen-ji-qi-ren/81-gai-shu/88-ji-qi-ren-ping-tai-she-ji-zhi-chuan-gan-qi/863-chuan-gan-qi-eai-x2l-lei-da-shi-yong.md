### 8.6.3 传感器\_EAI X2L雷达使用

EAI X2L激光雷达\(下图\)也是一款性价比较高的单线激光雷达。

![](/assets/EAI_X2L.jpg)

使用流程如下:

1. 硬件准备；
2. 软件安装；
3. 启动并测试。

#### 1.硬件准备

##### 1.雷达连接上位机

当前直接连接树莓派即可，如果连接的是虚拟机，注意VirtualBox或VMware的相关设置。

![](/assets/EAI雷达配置.PNG)

##### 2.确认当前的 USB 转串口终端并修改权限

USB查看命令:

```
ll /dev/ttyUSB*
```

授权\(将当前用户添加进dialout组，与arduino类似\):

```
sudo usermod -a -G dialout your_user_name
```

不要忘记重启，重启之后才可以生效。

#### 2.软件安装

##### 1.安装SDK

首先从github下载SDK安装包，命令如下:

```
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
```

然后安装，命令如下:

```
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```

##### 2.安装驱动

进入工作空间src目录，先下载驱动:

```
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
```

设置别名\(将端口 ttyUSBX 映射到 ydlidar\):

```
chmod 0777 ydlidar_ros_driver/startup/*
sudo sh ydlidar_ros_driver/startup/initenv.sh
```

返回工作空间目录，并调用`catkin_make`编译。

#### 3.启动并测试

根据雷达型号选择launch文件，当前雷达对应的是 X2.launch

##### 1.X2.launch文件准备

```xml
<launch>
  <node name="ydlidar_lidar_publisher"  pkg="ydlidar_ros_driver"  type="ydlidar_ros_driver_node" output="screen" respawn="false" >
    <!-- string property -->
    <param name="port"         type="string" value="/dev/ydlidar"/>  
    <param name="frame_id"     type="string" value="laser_frame"/>
    <param name="ignore_array"     type="string" value=""/>

    <!-- int property -->
    <param name="baudrate"         type="int" value="115200"/>  
    <!-- 0:TYPE_TOF, 1:TYPE_TRIANGLE, 2:TYPE_TOF_NET -->
    <param name="lidar_type"       type="int" value="1"/>  
    <!-- 0:YDLIDAR_TYPE_SERIAL, 1:YDLIDAR_TYPE_TCP -->
    <param name="device_type"         type="int" value="0"/>  
    <param name="sample_rate"         type="int" value="3"/>  
    <param name="abnormal_check_count"         type="int" value="4"/>  

    <!-- bool property -->
    <param name="resolution_fixed"    type="bool"   value="true"/>
    <param name="auto_reconnect"    type="bool"   value="true"/>
    <param name="reversion"    type="bool"   value="false"/>
    <param name="inverted"    type="bool"   value="true"/>
    <param name="isSingleChannel"    type="bool"   value="true"/>
    <param name="intensity"    type="bool"   value="false"/>
    <param name="support_motor_dtr"    type="bool"   value="true"/>
    <param name="invalid_range_is_inf"    type="bool"   value="false"/>
    <param name="point_cloud_preservative"    type="bool"   value="false"/>

    <!-- float property -->
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.1" />
    <param name="range_max"    type="double" value="12.0" />
    <!-- frequency is invalid, External PWM control speed -->
    <param name="frequency"    type="double" value="10.0"/>
  </node>
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
    args="0.0 0.0 0.2 0.0 0.0 0.0 /base_footprint /laser_frame 40" />

</launch>
```

frame\_id 需要按需修改，当使用URDF显示机器人模型时，需要与 URDF 中雷达 id 一致；launch 文件中静态坐标变换的节点可选用。

##### 2.终端中执行 launch 文件

终端命令:

```
roslaunch ydlidar_ros_driver X2.launch
```

##### 3.rviz中订阅雷达相关消息

启动 rviz，添加 LaserScan 插件：

![](/assets/EAI雷达.PNG)

注意: Fixed Frame 设置需要参考 X2.launch 中设置的 frame\_id，Topic 一般设置为 /scan，Size 可以自由调整。

---




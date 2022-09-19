### 8.6.2 传感器\_雷达使用

思岚A1激光雷达\(下图\)是一款性价比较高的单线激光雷达。

![](/assets/思岚A1%28新款%29.jpg)

使用流程如下:

1. 硬件准备；
2. 软件安装；
3. 启动并测试。

#### 1.硬件准备

##### 1.雷达连接上位机

当前直接连接树莓派即可，如果连接的是虚拟机，注意VirtualBox或VMware的相关设置。

![](/assets/VBox添加激光雷达.PNG)

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

进入工作空间的src目录，下载相关雷达驱动包，下载命令如下:

```
git clone https://github.com/slamtec/rplidar_ros
```

返回工作空间，调用`catkin_make`编译，并`source ./devel/setup.bash`，为端口设置别名\(将端口 ttyUSBX 映射到 rplidar\):

```
cd src/rplidar_ros/scripts/
./create_udev_rules.sh
```

#### 3.启动并测试

##### 1.rplidar.launch文件准备

首先确认端口,编辑 rplidar.launch 文件

```xml
<launch>
  <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="serial_port"         type="string" value="/dev/rplidar"/>
  <param name="serial_baudrate"     type="int"    value="115200"/><!--A1/A2 -->
  <!--param name="serial_baudrate"     type="int"    value="256000"--><!--A3 -->
  <param name="frame_id"            type="string" value="laser"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
</launch>
```

frame\_id 也可以修改，当使用URDF显示机器人模型时，需要与 URDF 中雷达 id 一致

##### 2.终端中执行 launch 文件

终端工作空间下输入命令:

```
roslaunch rplidar_ros rplidar.launch
```

如果异常，雷达开始旋转

##### 3.rviz中订阅雷达相关消息

启动 rviz，添加 LaserScan 插件：

![](/assets/思岚A1雷达显示.PNG)

注意: Fixed Frame 设置需要参考 rplidar.launch 中设置的 frame\_id，Topic 一般设置为 /scan，Size 可以自由调整。

---




### 8.6.5 传感器\_集成

之前已经分别介绍了底盘、雷达、相机等相关节点的安装、配置以及使用，不过之前的实现还存在一些问题:

> 1.机器人启动时，需要逐一启动底盘控制、相机与激光雷达，操作冗余；
>
> 2.如果只是简单的启动这些节点，那么在 rviz 中显示时，会发现出现了TF转换异常，比如参考坐标系设置为odom时，雷达信息显示失败。

本节将介绍如何把传感器\(激光雷达与相机\)集成以解决上述问题，所谓集成主要是优化底盘、雷达、相机相关节点的启动并通过坐标变换实现机器人底盘与里程计、雷达和相机的关联，实现步骤如下:

1. 编写用于集成的 launch 文件；
2. 发布TF坐标变换；
3. 启动并测试。

#### 1.launch文件

新建功能包:

```
catkin_create_pkg mycar_start roscpp rospy std_msgs ros_arduino_python usb_cam rplidar_ros
```

功能包下创建launch文件夹，launch文件夹中新建launch文件，文件名自定义。

内容如下:

```xml
<!-- 机器人启动文件：
        1.启动底盘
        2.启动激光雷达
        3.启动摄像头
 -->
<launch>
        <include file="$(find ros_arduino_python)/launch/arduino.launch" />
        <include file="$(find usb_cam)/launch/usb_cam-test.launch" />
        <include file="$(find rplidar_ros)/launch/rplidar.launch" />
</launch>
```

#### 2.坐标变换

如果启动时加载了机器人模型，且模型中设置的坐标系名称与机器人实体中设置的坐标系一致，那么可以不再添加坐标变换，因为机器人模型可以发布坐标变换信息，如果没有启动机器人模型，就需要自定义坐标变换实现了，继续新建launch文件。

内容如下:

```xml
<!-- 机器人启动文件：
        当不包含机器人模型时，需要发布坐标变换
 -->

<launch>
    <include file="$(find mycar_start)/launch/start.launch" />    
    <node name="camera2basefootprint" pkg="tf2_ros" type="static_transform_publisher" args="0.08 0 0.1 0 0 0 /base_footprint /camera_link"/>
    <node name="rplidar2basefootprint" pkg="tf2_ros" type="static_transform_publisher" args="0 0 0.1 0 0 0 /base_footprint /laser"/>
</launch>
```

#### 3.测试

最后，就可以启动PC端与树莓派端相关节点并运行查看结果了:

##### 1.树莓派

直接执行上一步的机器人启动launch文件:

```
roslaunch 自定义包 自定义launch文件
```

##### 2.PC端

启动键盘控制节点:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

还需要启动rviz:

```
rviz
```

##### 3.结果显示

在rviz中添加laserscan、image等插件，并通过键盘控制机器人运动，查看rviz中的显示结果:

![](/assets/机器人硬件集成测试.PNG)


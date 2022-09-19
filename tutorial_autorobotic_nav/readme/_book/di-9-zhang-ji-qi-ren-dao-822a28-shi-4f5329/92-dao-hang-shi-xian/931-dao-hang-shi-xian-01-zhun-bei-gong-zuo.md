### 9.3.1 导航实现01\_准备工作

#### 1.1分布式架构

分布式架构搭建完毕且能正常运行，在PC端可以远程登陆机器人端。

#### 1.2功能包安装

在机器人端安装导航所需功能包：

* 安装 gmapping 包\(用于构建地图\):`sudo apt install ros-<ROS版本>-gmapping`

* 安装地图服务包\(用于保存与读取地图\):`sudo apt install ros-<ROS版本>-map-server`

* 安装 navigation 包\(用于定位以及路径规划\):`sudo apt install ros-<ROS版本>-navigation`

新建功能（包名自定义，比如：nav），并导入依赖: gmapping map\_server amcl move\_base

#### 1.3机器人模型以及坐标变换

机器人的不同部件有不同的坐标系，我们需要将这些坐标系集成进同一坐标树，实现方案有两种：

1. 不同的部件相对于机器人底盘其位置都是固定的，可以通过发布静态坐标变换以实现集成；
2. 可以通过加载机器人URDF文件结合 robot\_state\_publisher、joint\_state\_publisher实现不同坐标系的集成。

方案1在上一章中已做演示，接下来介绍方案2的实现。

##### 1.3.1 创建机器人模型相关的功能包

创建功能包:`catkin_create_pkg mycar_description urdf xacro`。

##### 1.3.2 准备机器人模型文件

在功能包下新建 urdf 目录，编写具体的 urdf 文件（机器人模型相关URDF文件的编写可以参考第6章内容），示例如下：

文件car.urdf.xacro用于集成不同的机器人部件，内容如下：

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">

    <xacro:include filename="car_base.urdf.xacro" />
    <xacro:include filename="car_camera.urdf.xacro" />
    <xacro:include filename="car_laser.urdf.xacro" />

</robot>
```

文件car\_base.urdf.xacro机器人底盘实现，内容如下：

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">

    <xacro:property name="footprint_radius" value="0.001" />
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="${footprint_radius}" />
            </geometry>
        </visual>
    </link>


    <xacro:property name="base_radius" value="0.1" />
    <xacro:property name="base_length" value="0.08" />
    <xacro:property name="lidi" value="0.015" />
    <xacro:property name="base_joint_z" value="${base_length / 2 + lidi}" />
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.08" />
            </geometry>

            <origin xyz="0 0 0" rpy="0 0 0" />

            <material name="baselink_color">
                <color rgba="1.0 0.5 0.2 0.5" />
            </material>
        </visual>

    </link>

    <joint name="link2footprint" type="fixed">
        <parent link="base_footprint"  />
        <child link="base_link" />
        <origin xyz="0 0 0.055" rpy="0 0 0" />
    </joint>



    <xacro:property name="wheel_radius" value="0.0325" />
    <xacro:property name="wheel_length" value="0.015" />
    <xacro:property name="PI" value="3.1415927" />
    <xacro:property name="wheel_joint_z" value="${(base_length / 2 + lidi - wheel_radius) * -1}" />


    <xacro:macro name="wheel_func" params="wheel_name flag">

        <link name="${wheel_name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}" />
                </geometry>

                <origin xyz="0 0 0" rpy="${PI / 2} 0 0" />

                <material name="wheel_color">
                    <color rgba="0 0 0 0.3" />
                </material>
            </visual>

        </link>

        <joint name="${wheel_name}2link" type="continuous">
            <parent link="base_link"  />
            <child link="${wheel_name}_wheel" />

            <origin xyz="0 ${0.1 * flag} ${wheel_joint_z}" rpy="0 0 0" />
            <axis xyz="0 1 0" />
        </joint>

    </xacro:macro>

    <xacro:wheel_func wheel_name="left" flag="1" />
    <xacro:wheel_func wheel_name="right" flag="-1" />



    <xacro:property name="small_wheel_radius" value="0.0075" />
    <xacro:property name="small_joint_z" value="${(base_length / 2 + lidi - small_wheel_radius) * -1}" />

    <xacro:macro name="small_wheel_func" params="small_wheel_name flag">
        <link name="${small_wheel_name}_wheel">
            <visual>
                <geometry>
                    <sphere radius="${small_wheel_radius}" />
                </geometry>

                <origin xyz="0 0 0" rpy="0 0 0" />

                <material name="wheel_color">
                    <color rgba="0 0 0 0.3" />
                </material>
            </visual>

        </link>

        <joint name="${small_wheel_name}2link" type="continuous">
            <parent link="base_link"  />
            <child link="${small_wheel_name}_wheel" />

            <origin xyz="${0.08 * flag} 0 ${small_joint_z}" rpy="0 0 0" />
            <axis xyz="0 1 0" />
        </joint>

    </xacro:macro >
    <xacro:small_wheel_func small_wheel_name="front" flag="1"/>
    <xacro:small_wheel_func small_wheel_name="back" flag="-1"/>

</robot>
```

文件car\_camera.urdf.xacro机器人摄像头实现，内容如下：

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">

    <xacro:property name="camera_length" value="0.02" /> 
    <xacro:property name="camera_width" value="0.05" /> 
    <xacro:property name="camera_height" value="0.05" /> 
    <xacro:property name="joint_camera_x" value="0.08" />
    <xacro:property name="joint_camera_y" value="0" />
    <xacro:property name="joint_camera_z" value="${base_length / 2 + camera_height / 2}" />

    <link name="camera">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black">
                <color rgba="0 0 0 0.8" />
            </material>
        </visual>
    </link>

    <joint name="camera2base" type="fixed">
        <parent link="base_link" />
        <child link="camera" />
        <origin xyz="${joint_camera_x} ${joint_camera_y} ${joint_camera_z}" rpy="0 0 0" />
    </joint>

</robot>
```

文件car\_laser.urdf.xacro机器人雷达实现，内容如下：

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">

    <xacro:property name="support_radius" value="0.01" />
    <xacro:property name="support_length" value="0.15" />

    <xacro:property name="laser_radius" value="0.03" />
    <xacro:property name="laser_length" value="0.05" />

    <xacro:property name="joint_support_x" value="0" />
    <xacro:property name="joint_support_y" value="0" />
    <xacro:property name="joint_support_z" value="${base_length / 2 + support_length / 2}" />

    <xacro:property name="joint_laser_x" value="0" />
    <xacro:property name="joint_laser_y" value="0" />
    <xacro:property name="joint_laser_z" value="${support_length / 2 + laser_length / 2}" />

    <link name="support">
        <visual>
            <geometry>
                <cylinder radius="${support_radius}" length="${support_length}" />
            </geometry>
            <material name="yellow">
                <color rgba="0.8 0.5 0.0 0.5" />
            </material>
        </visual>

    </link>

    <joint name="support2base" type="fixed">
        <parent link="base_link" />
        <child link="support"/>
        <origin xyz="${joint_support_x} ${joint_support_y} ${joint_support_z}" rpy="0 0 0" />
    </joint>
    <link name="laser">
        <visual>
            <geometry>
                <cylinder radius="${laser_radius}" length="${laser_length}" />
            </geometry>
            <material name="black">
                <color rgba="0 0 0 0.5" />
            </material>
        </visual>

    </link>

    <joint name="laser2support" type="fixed">
        <parent link="support" />
        <child link="laser"/>
        <origin xyz="${joint_laser_x} ${joint_laser_y} ${joint_laser_z}" rpy="0 0 0" />
    </joint>
</robot>
```

##### 1.3.3 在launch文件加载机器人模型

launch 文件（文件名称自定义，比如：car.launch）内容示例如下：

```xml
<launch>
    <param name="robot_description" command="$(find xacro)/xacro $(find mycar_description)/urdf/car.urdf.xacro" />
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
</launch>
```

为了使用方便，还可以将该文件包含进启动机器人的launch文件中，示例如下：

```xml
<launch>
        <include file="$(find ros_arduino_python)/launch/arduino.launch" />
        <include file="$(find usb_cam)/launch/usb_cam-test.launch" />
        <include file="$(find rplidar_ros)/launch/rplidar.launch" />
        <!-- 机器人模型加载文件 -->
        <include file="$(find mycar_description)/launch/car.launch" />
</launch>
```

#### 1.4结果演示

不使用机器人模型时，机器人端启动机器人\(使用包含TF坐标换的launch文件\)，从机端启动rviz，在rviz中添加RobotModel与TF组件，rviz中结果\(此时显示机器人模型异常，且TF中只有代码中发布的坐标变换\):

![](/assets/TF坐标变换_静态.PNG)

使用机器人模型时，机器人端加载机器人模型（执行上一步的launch文件）且启动机器人，从机端启动rviz，，在rviz中添加RobotModel与TF组件rviz中结果\(此时显示机器人模型，且TF坐标变换正常\):

![](/assets/TF坐标变换_模型.PNG)

后续，在导航时使用机器人模型。

---




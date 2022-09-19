### 6.7.4 kinect信息仿真以及显示

通过 Gazebo 模拟kinect摄像头，并在 Rviz 中显示kinect摄像头数据。

**实现流程:**

kinect摄像头仿真基本流程:

1. 已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加kinect摄像头配置；

2. 将此文件集成进xacro文件；

3. 启动 Gazebo，使用 Rviz 显示kinect摄像头信息。

#### 1.Gazebo仿真Kinect

##### 1.1 新建 Xacro 文件，配置 kinetic传感器信息

```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
    <gazebo reference="kinect link名称">  
      <sensor type="depth" name="camera">
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <camera>
          <horizontal_fov>${60.0*PI/180.0}</horizontal_fov>
          <image>
            <format>R8G8B8</format>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
        <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
          <cameraName>camera</cameraName>
          <alwaysOn>true</alwaysOn>
          <updateRate>10</updateRate>
          <imageTopicName>rgb/image_raw</imageTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <frameName>kinect link名称</frameName>
          <baseline>0.1</baseline>
          <distortion_k1>0.0</distortion_k1>
          <distortion_k2>0.0</distortion_k2>
          <distortion_k3>0.0</distortion_k3>
          <distortion_t1>0.0</distortion_t1>
          <distortion_t2>0.0</distortion_t2>
          <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
      </sensor>
    </gazebo>

</robot>
```

##### 1.2 xacro 文件集成

将步骤1的 Xacro 文件集成进总的机器人模型文件，代码示例如下:

```xml
<!-- 组合小车底盘与传感器 -->
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_head.urdf.xacro" />
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
    <xacro:include filename="move.urdf.xacro" />
    <!-- kinect仿真的 xacro 文件 -->
    <xacro:include filename="my_sensors_kinect.urdf.xacro" />
</robot>
```

##### 1.3启动仿真环境

编写launch文件，启动gazebo，此处略...

#### 2 Rviz 显示 Kinect 数据

启动 rviz,添加摄像头组件查看数据![](/assets/kinect摄像头.PNG)![](/assets/16_kinect仿真.png)

---

#### **补充:kinect 点云数据显示**

在kinect中也可以以点云的方式显示感知周围环境，在 rviz 中操作如下:

![](/assets/点云数据_默认.PNG)

**问题:**在rviz中显示时错位。

**原因:**在kinect中图像数据与点云数据使用了两套坐标系统，且两套坐标系统位姿并不一致。

**解决:**

1.在插件中为kinect设置坐标系，修改配置文件的`<frameName>`标签内容：

```xml
<frameName>support_depth</frameName>
```

2.发布新设置的坐标系到kinect连杆的坐标变换关系，在启动rviz的launch中，添加:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="static_transform_publisher" args="0 0 0 -1.57 0 -1.57 /support /support_depth" />
```

3.启动rviz，重新显示。

![](/assets/点云数据_修正.PNG)


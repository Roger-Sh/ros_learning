### 8.6.4 传感器\_相机使用

使用流程如下:

1. 硬件准备；
2. 软件安装；
3. 启动并测试。

#### 1.硬件准备

当前直接连接树莓派即可，如果连接的是虚拟机，注意VirtualBox或VMware的相关设置。

![](/assets/摄像头配置.PNG)

#### 2.软件准备

安装USB摄像头软件包，命令如下:

```
sudo apt-get install ros-ROS版本-usb-cam
```

或者也可以从 github 直接下载源码:

```
git clone https://github.com/ros-drivers/usb_cam.git
```

#### 3.测试

##### 1.launch文件准备

在软件包中内置了测试用的launch文件，内容如下:

```xml
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device" value="/dev/video0" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
  <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
    <remap from="image" to="/usb_cam/image_raw"/>
    <param name="autosize" value="true" />
  </node>
</launch>
```

节点 usb\_cam 用于启动相机，节点 image\_view 以图形化窗口的方式显示图像数据，需要查看相机的端口并修改 usb\_cam 中的 video\_device 参数，并且如果将摄像头连接到了树莓派，且通过 ssh 远程访问树莓派的话，需要注释 image\_view 节点，因为在终端中无法显示图形化界面。

##### 2.启动launch文件

```
roslaunch usb_cam usb_cam-test.launch
```

##### 3.rviz显示

启动 rviz，添加 LaserScan 插件：

![](/assets/相机显示.PNG)

---




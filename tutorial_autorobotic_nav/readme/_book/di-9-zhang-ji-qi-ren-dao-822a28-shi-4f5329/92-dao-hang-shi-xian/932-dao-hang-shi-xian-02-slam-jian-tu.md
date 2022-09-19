### 9.3.2 导航实现02\_SLAM建图

关于建图实现，仍然选用第7章中学习的：gmapping，实现如下：

#### 2.1编写gmapping节点相关launch文件

在上一节创建的导航功能包中新建 launch 目录，并新建launch文件（文件名自定义，比如： gmapping.launch），代码示例如下:

```xml
<launch>
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
      <remap from="scan" to="scan"/>
      <param name="base_frame" value="base_footprint"/><!--底盘坐标系-->
      <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="16.0"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>
</launch>
```

关键代码解释：

```xml
<remap from="scan" to="scan"/><!-- 雷达话题 -->
<param name="base_frame" value="base_footprint"/><!--底盘坐标系-->
<param name="odom_frame" value="odom"/> <!--里程计坐标系-->
```

#### 2.2执行

1.执行相关launch文件，启动机器人并加载机器人模型：`roslaunch mycar_start start.launch`；

2.启动地图绘制的 launch 文件：`roslaunch nav gmapping.launch`；

3.启动键盘键盘控制节点，用于控制机器人运动建图：`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

4.在 rviz 中添加地图显示组件，通过键盘控制机器人运动，同时，在rviz中可以显示gmapping发布的栅格地图数据了，该显示结果与仿真环境下类似。下一步，还需要将地图单独保存。


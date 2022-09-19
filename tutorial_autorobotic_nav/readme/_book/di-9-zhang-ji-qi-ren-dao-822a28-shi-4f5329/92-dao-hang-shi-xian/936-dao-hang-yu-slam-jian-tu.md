### 9.3.6 导航与SLAM建图

与仿真环境类似的，也可以实现机器人自主移动的SLAM建图，步骤如下：

1. 编写launch文件，集成SLAM与move\_base相关节点；
2. 执行launch文件并测试。

#### 6.1编写launc文件

当前launch文件（名称自定义，比如：auto\_slam.launch）实现，无需调用map\_server的相关节点，只需要启动SLAM节点与move\_base节点，示例内容如下:

```xml
<launch>
    <!-- 启动SLAM节点 -->
    <include file="$(find nav)/launch/gmapping.launch" />
    <!-- 运行move_base节点 -->
    <include file="$(find nav)/launch/move_base.launch" />
</launch>
```

#### 6.2测试

1.执行相关launch文件，启动机器人并加载机器人模型：`roslaunch mycar_start start.launch`；

2.然后执行当前launch文件：`roslaunch nav auto_slam.launch`；

3.在rviz中通过2D Nav Goal设置目标点，机器人开始自主移动并建图了；

4.最后可以使用 map\_server 保存地图。


### 9.3.3 导航实现03\_地图服务

可以通过 map\_server 实现地图的保存与读取。

#### 3.1地图保存launch文件

首先在自定义的导航功能包下新建 map 目录，用于保存生成的地图数据。地图保存的语法比较简单，编写一个launch文件，内容如下:

```xml
<launch>
    <arg name="filename" value="$(find nav)/map/nav" />
    <node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```

其中通过 filename 指定了地图的保存路径以及保存的文件名称。

SLAM建图完毕后，执行该launch文件即可。

**测试:**

首先，参考上一节，依次启动仿真环境，键盘控制节点与SLAM节点；

然后，通过键盘控制机器人运动并绘图；

最后，通过上述地图保存方式保存地图。

结果：在指定路径下会生成两个文件，xxx.pgm 与 xxx.yaml

#### 3.2地图读取

通过 map\_server 的 map\_server 节点可以读取栅格地图数据，编写 launch 文件如下:

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find mycar_nav)/map/$(arg map)"/>
</launch>
```

其中参数是地图描述文件的资源路径，执行该launch文件，该节点会发布话题:map\(nav\_msgs/OccupancyGrid\)，最后，在 rviz 中使用 map 组件可以显示栅格地图。




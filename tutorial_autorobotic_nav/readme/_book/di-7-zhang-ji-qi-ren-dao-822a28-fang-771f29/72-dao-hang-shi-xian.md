## 7.2 导航实现

本节内容主要介绍导航的完整性实现，旨在掌握机器人导航的基本流程，该章涉及的主要内容如下:

* SLAM建图\(选用较为常见的gmapping\)

* 地图服务\(可以保存和重现地图\)

* 机器人定位

* 路径规划

* 上述流程介绍完毕，还会对功能进一步集成实现探索式的SLAM建图。

---

**准备工作**

请先安装相关的ROS功能包:

* 安装 gmapping 包\(用于构建地图\):`sudo apt install ros-<ROS版本>-gmapping`

* 安装地图服务包\(用于保存与读取地图\):`sudo apt install ros-<ROS版本>-map-server`

* 安装 navigation 包\(用于定位以及路径规划\):`sudo apt install ros-<ROS版本>-navigation`

新建功能包，并导入依赖: gmapping map\_server amcl move\_base

---




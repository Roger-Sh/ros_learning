### 7.3.1 导航之地图

地图相关的消息主要有两个:

nav\_msgs/MapMetaData

* 地图元数据，包括地图的宽度、高度、分辨率等。

nav\_msgs/OccupancyGrid

* 地图栅格数据，一般会在rviz中以图形化的方式显示。

#### 1.nav\_msgs/MapMetaData

调用`rosmsg info nav_msgs/MapMetaData`显示消息内容如下:

```
time map_load_time
float32 resolution #地图分辨率
uint32 width #地图宽度
uint32 height #地图高度
geometry_msgs/Pose origin #地图位姿数据
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

#### 2.nav\_msgs/OccupancyGrid

调用 `rosmsg info nav_msgs/OccupancyGrid`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
#--- 地图元数据
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
#--- 地图内容数据，数组长度 = width * height
int8[] data
```




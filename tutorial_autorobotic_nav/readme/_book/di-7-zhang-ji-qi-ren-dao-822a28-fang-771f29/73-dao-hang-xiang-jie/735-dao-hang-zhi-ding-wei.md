### 7.3.4 导航之定位

定位相关消息是:geometry\_msgs/PoseArray，调用`rosmsg info geometry_msgs/PoseArray`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] poses #预估的点位姿组成的数组
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




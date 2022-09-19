### 7.3.6 导航之激光雷达

激光雷达相关消息是:sensor\_msgs/LaserScan，调用`rosmsg info sensor_msgs/LaserScan`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min #起始扫描角度(rad)
float32 angle_max #终止扫描角度(rad)
float32 angle_increment #测量值之间的角距离(rad)
float32 time_increment #测量间隔时间(s)
float32 scan_time #扫描间隔时间(s)
float32 range_min #最小有效距离值(m)
float32 range_max #最大有效距离值(m)
float32[] ranges #一个周期的扫描数据
float32[] intensities #扫描强度数据，如果设备不支持强度数据，该数组为空
```




### 7.3.7 导航之相机

深度相机相关消息有:sensor\_msgs/Image、sensor\_msgs/CompressedImage、sensor\_msgs/PointCloud2

sensor\_msgs/Image 对应的一般的图像数据，sensor\_msgs/CompressedImage 对应压缩后的图像数据，sensor\_msgs/PointCloud2 对应的是点云数据\(带有深度信息的图像数据\)。

调用`rosmsg info sensor_msgs/Image`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height #高度
uint32 width  #宽度
string encoding #编码格式:RGB、YUV等
uint8 is_bigendian #图像大小端存储模式
uint32 step #一行图像数据的字节数，作为步进参数
uint8[] data #图像数据，长度等于 step * height
```

调用`rosmsg info sensor_msgs/CompressedImage`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string format #压缩编码格式(jpeg、png、bmp)
uint8[] data #压缩后的数据
```

调用`rosmsg info sensor_msgs/PointCloud2`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height #高度
uint32 width  #宽度
sensor_msgs/PointField[] fields #每个点的数据类型
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count
bool is_bigendian #图像大小端存储模式
uint32 point_step #单点的数据字节步长
uint32 row_step   #一行数据的字节步长
uint8[] data      #存储点云的数组，总长度为 row_step * height
bool is_dense     #是否有无效点
```




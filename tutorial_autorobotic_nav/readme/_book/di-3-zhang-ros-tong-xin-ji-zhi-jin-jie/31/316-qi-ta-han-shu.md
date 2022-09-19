### 3.1.5 其他函数

在发布实现时，一般会循环发布消息，循环的判断条件一般由节点状态来控制，C++中可以通过 ros::ok\(\) 来判断节点状态是否正常，而 python 中则通过 rospy.is\_shutdown\(\) 来实现判断，导致节点退出的原因主要有如下几种:

* 节点接收到了关闭信息，比如常用的 ctrl + c 快捷键就是关闭节点的信号；
* 同名节点启动，导致现有节点退出；
* 程序中的其他部分调用了节点关闭相关的API\(C++中是ros::shutdown\(\)，python中是rospy.signal\_shutdown\(\)\)

另外，日志相关的函数也是极其常用的，在ROS中日志被划分成如下级别:

* DEBUG\(调试\):只在调试时使用，此类消息不会输出到控制台；
* INFO\(信息\):标准消息，一般用于说明系统内正在执行的操作；
* WARN\(警告\):提醒一些异常情况，但程序仍然可以执行；
* ERROR\(错误\):提示错误信息，此类错误会影响程序运行；
* FATAL\(严重错误\):此类错误将阻止节点继续运行。

---

#### C++

1.节点状态判断

```cpp
/** \brief 检查节点是否已经退出
 *
 *  ros::shutdown() 被调用且执行完毕后，该函数将会返回 false
 *
 * \return true 如果节点还健在, false 如果节点已经火化了。
 */
bool ok();
```

2.节点关闭函数

```cpp
/*
*   关闭节点
*/
void shutdown();
```

3.日志函数

使用示例

```cpp
ROS_DEBUG("hello,DEBUG"); //不会输出
ROS_INFO("hello,INFO"); //默认白色字体
ROS_WARN("Hello,WARN"); //默认黄色字体
ROS_ERROR("hello,ERROR");//默认红色字体
ROS_FATAL("hello,FATAL");//默认红色字体
```

---

#### Python

1.节点状态判断

```py
def is_shutdown():
    """
    @return: True 如果节点已经被关闭
    @rtype: bool
    """
```

2.节点关闭函数

```py
def signal_shutdown(reason):
    """
    关闭节点
    @param reason: 节点关闭的原因，是一个字符串
    @type  reason: str
    """
```

```py
def on_shutdown(h):
    """
    节点被关闭时调用的函数
    @param h: 关闭时调用的回调函数，此函数无参
    @type  h: fn()
    """
```

3.日志函数

使用示例

```py
rospy.logdebug("hello,debug")  #不会输出
rospy.loginfo("hello,info")  #默认白色字体
rospy.logwarn("hello,warn")  #默认黄色字体
rospy.logerr("hello,error")  #默认红色字体
rospy.logfatal("hello,fatal") #默认红色字体
```




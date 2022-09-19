## 10.4 nodelet

ROS通信是基于Node\(节点\)的，Node使用方便、易于扩展，可以满足ROS中大多数应用场景，但是也存在一些局限性，由于一个Node启动之后独占一根进程，不同Node之间数据交互其实是不同进程之间的数据交互，当传输类似于图片、点云的大容量数据时，会出现延时与阻塞的情况，比如：

> 现在需要编写一个相机驱动，在该驱动中有两个节点实现:其中节点A负责发布原始图像数据，节点B订阅原始图像数据并在图像上标注人脸。如果节点A与节点B仍按照之前实现，两个节点分别对应不同的进程，在两个进程之间传递容量可观图像数据，可能就会出现延时的情况，那么该如何优化呢？

ROS中给出的解决方案是:Nodelet，通过Nodelet可以将多个节点集成进一个进程。

---

#### **概念**

nodelet软件包旨在提供在同一进程中运行多个算法\(节点\)的方式，不同算法之间通过传递指向数据的指针来代替了数据本身的传输\(类似于编程传值与传址的区别\)，从而实现零成本的数据拷贝。

nodelet功能包的核心实现也是插件，是对插件的进一步封装:

* 不同算法被封装进插件类，可以像单独的节点一样运行；
* 在该功能包中提供插件类实现的基类:Nodelet；
* 并且提供了加载插件类的类加载器:NodeletLoader。

#### **作用**

应用于大容量数据传输的场景，提高节点间的数据交互效率，避免延时与阻塞。

---

**另请参考:**

* [http://wiki.ros.org/nodelet/](http://wiki.ros.org/nodelet/)

* [http://wiki.ros.org/nodelet/Tutorials/Running%20a%20nodelet](http://wiki.ros.org/nodelet/Tutorials/Running a nodelet)

* [https://github.com/ros/common\_tutorials/tree/noetic-devel/nodelet\_tutorial\_math](https://github.com/ros/common_tutorials/tree/noetic-devel/nodelet_tutorial_math)




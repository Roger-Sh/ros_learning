**ROS 多线程笔记**

* **Topic**
  * **pub**
    * **一般在main loop中配合 ros::Rate 使用**
  * **sub**
    * **spinOnce模式下，同一节点中如果存在多个subscriber callback，其中一个callback耗时会阻塞其他callback以及main loop中的其他操作。**
    * **AsyncSpinner模式下，同一节点中如果存在多个subscriber callback，可以通过设置多个线程来避免耗时操作的阻塞影响**
      * **AsyncSpinner的多线程数量只针对callback，main中后续的操作一直不受影响，哪怕多个callback有耗时操作，多线程设为1，主线程也不影响**
      * **AsyncSpinner 多线程数量设置为0表示根据CPU核心数来选择线程数**
      * **AsyncSpinner的多线程数量取决于同一时间可能有多少耗时操作，尽量避免在callback中进行耗时操作**
      * **callback 中如果存在共享资源，则需要加Mutex锁避免资源竞争问题**
* **Service**
  * **server**
    * **callback**
      * **server callback 通过 AsyncSpinner 实现多线程响应, 多线程数量设置为0可以根据CPU核心数来分配线程，效率最高**
      * **尽量避免server中使用耗时操作, 耗时的操作可以使用action server**
  * **client**
    * **ROS client 没有 non-blocking 的方式**
    * **尽量避免client调用太耗时**
    * **如果必须要使用非阻塞式的方式使用client，可以通过std::thread新建一个线程，不过要注意线程数量，防止线程数量爆炸式增长。最好一次只调用一次。**
* **Action**
  * **server**
    * **callback**
      * **是否影响主线程**
      * **是否影响其他callback**
  * **client**
    * **client call blocking**
    * **client call async with callback**

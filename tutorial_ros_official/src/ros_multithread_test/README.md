# ROS 多线程笔记

## Topic

* pub
  * 一般在main loop中配合 ros::Rate 使用
* sub
  * spinOnce模式下，同一节点中如果存在多个subscriber callback，其中一个callback耗时会阻塞其他callback以及main loop中的其他操作。
  * AsyncSpinner模式下，同一节点中如果存在多个subscriber callback，可以通过设置多个线程来避免耗时操作的阻塞影响
    * AsyncSpinner的多线程数量只针对callback，main_thread不受影响，哪怕多个callback有耗时操作，AsyncSpinner多线程设为1，主线程也不影响
    * AsyncSpinner 多线程数量设置为0表示根据CPU核心数来选择线程数
    * AsyncSpinner的多线程数量取决于同一时间可能有多少耗时操作，尽量避免在callback中进行耗时操作
    * callback 中如果存在共享资源，则需要加Mutex锁避免资源竞争问题



## Service

* server
  * callback
    * server callback 通过 AsyncSpinner 实现多线程响应, 多线程数量设置为0可以根据CPU核心数来分配线程，效率最高
    * 尽量避免server中使用耗时操作, 耗时的操作可以使用action server
* client
  * ROS client 没有 non-blocking 的方式，尽量避免client调用太耗时
  * 如果必须要使用非阻塞式的方式使用client，可以通过std::thread新建一个线程，不过要注意线程数量，防止线程数量爆炸式增长。最好一个client同时只调用一次。



## Action

* client
  * waitForServer()
    * 是阻塞式的
    * 单线程模式下该方法不会有效果，因为主线程被阻塞，将无法得到server端的回应
      * 单线程模式指client初始化时没有将spin_thread设置为true，或者没有任何spin操作
      * action client 初始化时将spin_thread 设置为true, 或者在 action cleint 初始化之前使用Asycspinner，可以收到server连接的回应。
  * sendGoal()
    * 可以绑定callbacks (active, feedback, done), 显示 action 的各个状态
      * 同一个client连续发送时，第二次会征用第一次的callback，导致第一次的callback无法接收反馈与结果，而第二次的callback也会等server端第一次完成后才开始接收第二次的反馈与结果。
    * 也可以不绑定，通过waitForResult获取结果
  * waitForResult()
    * 是阻塞式的
    * 可以不使用waitForResult, 通过绑定callback响应动作的反馈消息和结果消息。这样便可以使client的使用不阻塞。
* server
  * ros::spin()模式 与 ros::Asyncspinner 模式对server端效果相同
  * 同一个节点内同一个server会顺序执行收到的消息

    * client端，同一个client连续发送，第二次会打断第一次的callback，因为同一个callback被第二次呼叫征用了，所以第一次的callback不会再起作用，而第二次的callback会等server端把第一次的动作执行完成之后，才会开始收到第二次的反馈
    * server端，单线程按顺序执行
  * 同一个节点内不同server，可以同时响应消息

    * client端，不同client同时呼叫不同server，client端会多线程响应。
    * server端不同server多线程响应。比如server1被耗时操作阻塞，server2仍能响应。

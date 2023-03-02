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
  
  * ros::spinOnce() 单线程模式
    * server收到请求后，耗时操作会阻塞主线程
    * 同一个节点，单个server，在单线程模式下同时只能响应一次，多次请求会根据顺序依次响应
    * 同一个节点，多个server，在单线程模式下同时只能有一个server作出响应，不同server接收的请求按照顺序执行
  
  
  * ros::AsyncSpinner 多线程模式
    * server收到请求后，耗时操作不会阻塞主线程
    * 同一个节点，单个server，在多线程模式下可以同时响应 
        * **这种模式有冲突隐患，可以在server端设置锁，防止同一个server同时响应多个client的请求**
        * **同一个节点内需要不间断发布状态时仍然有必要采用多线程的server模式**
  
    * 同一个节点，多个server，在多线程模式下可以同时响应
    * 多线程数量设置为0可以根据CPU核心数来分配线程，效率最高
    * 尽量避免server中使用耗时操作, 耗时的操作推荐使用 action server
  
* client
  * ROS client 没有非阻塞式的方式的方式，尽量避免client调用太耗时
  * 如果必须要使用非阻塞式的方式使用client，可以通过std::thread新建一个线程，不过要注意线程数量，防止线程数量爆炸式增长。最好一个client同时只调用一次。



## Action

* action client
  * waitForServer()
    * 是阻塞式的
    * 可以在程序初始阶段统一开始连接
    * 单线程模式下该方法不会有效果，因为主线程被阻塞，将无法得到server端的回应
      * 单线程模式指client初始化时没有将spin_thread设置为true，或者没有任何spin操作
      * action client 初始化时将spin_thread 设置为true, 或者在 action cleint 初始化之前使用Asycspinner，可以收到server连接的回应。
  * sendGoal()
    * 可以绑定callbacks (active, feedback, done), 显示 action 的各个状态
      * 同一个client连续发送时，第二次会征用第一次的callback，导致第一次的callback无法接收反馈与结果，而第二次的callback也会等server端第一次完成后才开始接收第二次的反馈与结果。
    * 也可以不绑定，通过waitForResult获取结果
  * waitForResult()
    * 是阻塞式的
    * 不阻塞使用client的方法
        * 不使用waitForResult, 通过绑定callback响应动作的反馈消息和结果消息。
* action server
  * ros::spinOnce()模式 与 ros::Asyncspinner 模式
      * 同一节点内不同server之间是多线程的
      * 同一节点内同一server是单线程的
      * 均不会阻塞主线程
      * Asyncspinner模式下，同一个server可以接收不同的请求，按顺序执行
      * spinOnce模式下，同一个server的同一时刻只能接收一个请求，这里的同一时刻取决于spinOnce的频率
  * 同一个节点内同一个server会顺序执行收到的消息
  
    * client端，同一个client连续发送，第二次会打断第一次的callback，因为同一个callback被第二次呼叫征用了，所以第一次的callback不会再起作用，而第二次的callback会等server端把第一次的动作执行完成之后，才会开始收到第二次的反馈
    * server端，同一server一直按照单线程顺序执行
  * 同一个节点内不同server，可以同时响应消息
  
    * client端，不同client同时呼叫不同server，client端会多线程响应。
    * server端不同server多线程响应。比如server1被耗时操作阻塞，server2仍能响应。



-   总结
    -   如果一个节点在不间断发布消息的同时
        -   需要作为 service server 或 action server 提供服务
            -   使用Asyncspinner开启多线程模式
            -   可以通过加锁的方式避免同时重复调用同一个server
        -   需要使用有可能产生堵塞的 service client，
            -   通过临时增加一个线程，但要注意不重复调用
        -   需要使用有可能产生堵塞的 action client
            -   通过callback处理反馈信息，不使用waitForResult()


### 3.1.3 回旋函数

---

#### C++

在ROS程序中，频繁的使用了 ros::spin\(\) 和 ros::spinOnce\(\) 两个回旋函数，可以用于处理回调函数。

##### 1.spinOnce\(\)

```cpp
/**
 * \brief 处理一轮回调
 *
 * 一般应用场景:
 *     在循环体内，处理所有可用的回调函数
 * 
 */
ROSCPP_DECL void spinOnce();
```

##### 2.spin\(\)

```cpp
/** 
 * \brief 进入循环处理回调 
 */
ROSCPP_DECL void spin();
```

##### 3.二者比较

**相同点:**二者都用于处理回调函数；

**不同点:**ros::spin\(\) 是进入了循环执行回调函数，而 ros::spinOnce\(\) 只会执行一次回调函数\(没有循环\)，在 ros::spin\(\) 后的语句不会执行到，而 ros::spinOnce\(\) 后的语句可以执行。

---

#### Python

```py
def spin():
    """
    进入循环处理回调 
    """
```




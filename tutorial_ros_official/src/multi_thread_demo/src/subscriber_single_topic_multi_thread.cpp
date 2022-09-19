#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief 假设只有一个Topic, 发布端的频率比较高，
 * 我们又想尽可能多地响应消息，因此我们可以设置多个Spinner，
 * 但是单纯地像上一小节一样使用MultiThreadedSpinner是不行的，
 * ros作了限制，默认阻止并行处理一个Callback，我们需要更改Suscriber的配置,
 * 通过 allow_concurrent_callbacks 可以设置同一个callback被不同的线程调用
 * 这里设置ChatterCallback每收到一个消息, sleep 10s, 
 * 如果spinner只有一个线程, 那么搁10s才能处理1个消息
 * 如果spinner有2个线程, 那么每隔10s可以处理2个消息
 * 如果spinner有10个线程, 那么每隔10s可以处理10个消息
 */



void ChatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO(" I heard: [%s]", msg->data.c_str());

    // sleep 2s
    ROS_INFO(" sleep for 10s");
    std::this_thread::sleep_for(std::chrono::seconds(10));
}

int main(int argc, char **argv)
{
    // init ros
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    // init subscriber with ops
    ros::SubscribeOptions ops;
    ops.init<std_msgs::String>("/A/message", 1, ChatterCallback);
    ops.allow_concurrent_callbacks = true; // 同一个subscriber可以在多个线程执行

    // subscribe
    ros::Subscriber sub1 = n.subscribe(ops);

    // set MultiThreadedSpinner 
    ros::MultiThreadedSpinner spinner(2);

    // ros spin
    spinner.spin();
    return 0;
}
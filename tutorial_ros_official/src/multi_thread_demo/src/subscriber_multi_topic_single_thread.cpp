#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief 接受multi_publisher.launch发布的两个消息, 消息发布频率为1HZ
 * 这个程序中 CallbackA 处理消息 /A/message 时 sleep 2s
 * CallbackB 没有间隔
 * 执行结果 CallBackB 的处理频率也变成2s一次
 * 在只有一个Spinner thread的情况下，callback queue只能顺序执行。
 * 这就说明了单线程的不足，不管有多少个Subscriber，
 * 节点都只能顺序执行回调，这在某些时候是不能忍受的，
 * 因此，多线程有了用武之地，我们要做的事情就是增加spinner thread。
 */


void CallbackA(const std_msgs::String::ConstPtr &msg)
{
    // sleep 2s
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ROS_INFO(" I heard: [%s]", msg->data.c_str());
}

void CallbackB(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO(" I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_multi_topic_single_thread");
    ros::NodeHandle n;
    ros::Subscriber sub_b = n.subscribe("/B/message", 1, CallbackB);
    ros::Subscriber sub_a = n.subscribe("/A/message", 1, CallbackA);
    
    ros::spin();
    return 0;
}
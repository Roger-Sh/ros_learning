#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief CallbackB处理了B/message中的所有消息(1,2,3,4,5,6)，
 * 而CallbackA还是2s调用一次，只处理了A/message中编号为1，3，5的消息。
 * 也就是说，我们有一个空闲的线程在另一个线程被CallbackA占用时可以从容地处理CallbackB。
 */


void CallbackA(const std_msgs::String::ConstPtr &msg)
{
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ROS_INFO(" I heard: [%s]", msg->data.c_str());
}

void CallbackB(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO(" I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_multi_topic_multi_thread");
    ros::NodeHandle n;
    ros::Subscriber sub_b = n.subscribe("/B/message", 1, CallbackB);
    ros::Subscriber sub_a = n.subscribe("/A/message", 1, CallbackA);
    
    // 通过 MultiThreadedSpinner 开启两个线程
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
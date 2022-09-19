#include <thread>
#include <ros/callback_queue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief 给每个 subscriber 指定独立的 thread, 独立的 callback queue
 * 
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
    // init ros
    ros::init(argc, argv, "listener");

    // 绑定 sub_b, CallbackB 到 n_b
    ros::NodeHandle n_b;
    ros::Subscriber sub_b = n_b.subscribe("/A/message", 1, CallbackB);

    // 绑定 sub_a, CallbackA 到 n_a
    ros::NodeHandle n_a;
    ros::CallbackQueue callback_queue_a;
    n_a.setCallbackQueue(&callback_queue_a);
    ros::Subscriber sub_a = n_a.subscribe("/B/message", 1, CallbackA);
    std::thread spinner_thread_a(
        [&callback_queue_a]()
        {
            ros::SingleThreadedSpinner spinner_a;
            spinner_a.spin(&callback_queue_a); 
        }
    );

    // ros spin
    ros::spin();
    spinner_thread_a.join();
    return 0;
}
/**
 * @file multiListenerClassMultiThreadedSpinner.cpp
 * @author Weipu Shan (weipu.shan@foxmail.com)
 * @brief test multi thread subscriber in class using MultiThreadedSpinner
 * @version 0.1
 * @date 2022-09-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread.hpp>

/**
 * @brief multiThreadListener
 *
 */
class multiListener
{
public:
    // constructor
    multiListener()
    {
        sub1 = n.subscribe("/A/message", 1, &multiListener::chatterCallback1, this);
        sub2 = n.subscribe("/B/message", 1, &multiListener::chatterCallback2, this);
    }

    // callbacks
    void chatterCallback1(const std_msgs::String::ConstPtr &msg);
    void chatterCallback2(const std_msgs::String::ConstPtr &msg);

private:
    ros::NodeHandle n;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
};

/**
 * @brief chatterCallback1
 * 
 * @param msg 
 */
void multiListener::chatterCallback1(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    ros::Rate loop_rate(0.5); // block chatterCallback2()
    loop_rate.sleep();
}

/**
 * @brief chatterCallback2
 * 
 * @param msg 
 */
void multiListener::chatterCallback2(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @brief main
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "multiListenerClassMultiThreadedSpinner");

    multiListener listener_obj;
    ros::MultiThreadedSpinner s(2);

    ros::spin(s);

    return 0;
}

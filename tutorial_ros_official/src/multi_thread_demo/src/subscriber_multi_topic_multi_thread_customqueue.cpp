/**
 * @file subscriber_multi_topic_multi_thread_customqueue.cpp
 * @author your name (you@domain.com)
 * @brief This tutorial demonstrates the use of custom separate callback queues that can be processed
 * independently, whether in different threads or just at different times.
 * @version 0.1
 * @date 2022-09-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>


/**
 * @brief This callback gets called from the main queue processed in spin()
 * 
 * @param msg 
 */
void chatterCallbackMainQueue(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread [" << boost::this_thread::get_id() << "] (Main thread)");
}

/**
 * @brief This callback gets called from the custom queue
 * 
 * @param msg 
 */
void chatterCallbackCustomQueue(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread [" << boost::this_thread::get_id() << "]");
}

/**
 * @brief The custom queue used for one of the subscription callbacks
 * 
 */
ros::CallbackQueue g_queue;

/**
 * @brief a callbaclThread to process custome queue
 * 
 */
void callbackThread()
{
    ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

    // here init a new ros nodehandle to process custom queue
    ros::NodeHandle n;
    while (n.ok())
    {
        // set custom queue timout
        g_queue.callAvailable(ros::WallDuration(10));  
    }
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
    // init ros
    ros::init(argc, argv, "multi_topic_multi_thread_customqueue");
    ros::NodeHandle n;

    /**
     * @brief The SubscribeOptions structure lets you specify a custom queue to use for a specific subscription.
     * You can also set a default queue on a NodeHandle using the NodeHandle::setCallbackQueue() function.
     * AdvertiseOptions and AdvertiseServiceOptions offer similar functionality.
     */

    // sub ops with cumtom queue
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::String>(
        "/A/message", 1,
        chatterCallbackCustomQueue,
        ros::VoidPtr(), &g_queue);
    // sub with sub ops
    ros::Subscriber sub = n.subscribe(ops);
    // init a new thread for custom queue
    boost::thread chatter_thread(callbackThread);

    /**
     * @brief default main queue
     * 
     */
    ros::Subscriber sub2 = n.subscribe("/B/message", 1, chatterCallbackMainQueue);

    /**
     * @brief ros loop only effects main queue, no effect for custom queue
     * 
     */
    ros::Rate r(0.1);
    while (n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    chatter_thread.join();

    return 0;
}

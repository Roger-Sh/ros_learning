/**
 * @file multiListenerClassAsyncSpinner.cpp
 * @author Weipu Shan (weipu.shan@foxmail.com)
 * @brief test multi thread subscriber in class using AsyncSpinner
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
class multiThreadListener
{
public:
	multiThreadListener()
	{	
		sub1 = n.subscribe("/A/message", 1, &multiThreadListener::chatterCallback1,this);
		sub2 = n.subscribe("/B/message", 1, &multiThreadListener::chatterCallback2,this);
	}
	void chatterCallback1(const std_msgs::String::ConstPtr& msg);
	void chatterCallback2(const std_msgs::String::ConstPtr& msg);

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
void multiThreadListener::chatterCallback1(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  ros::Rate loop_rate(0.5);//block chatterCallback2()
  loop_rate.sleep();
}

/**
 * @brief chatterCallback2
 * 
 * @param msg 
 */
void multiThreadListener::chatterCallback2(const std_msgs::String::ConstPtr& msg)
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

  ros::init(argc, argv, "mulyiThreadListenerClassAsyncSpinner");

  multiThreadListener listener_obj;
  
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}



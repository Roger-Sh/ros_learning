#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>



int main (int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "test_fibonacci");

    // create the action client
    // 创建一个action client
    // msg type: actionlib_tutorials::FibonacciAction, server name: "fibonacci", true: causes the client to spin its own thread
    actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);
    
    // wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); 
    ROS_INFO("Action server started, sending goal.");
    
    // send a goal to the action
    actionlib_tutorials::FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal(goal);

    // wait for the action to return
    // 等待server回传数据，得到一个flag， 判断是否在限时内结束
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        // 得到 server回传的状态
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
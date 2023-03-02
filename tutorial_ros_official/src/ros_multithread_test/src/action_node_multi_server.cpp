#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "ros_multithread_test/CountNumAction.h"

class ActionNodeMultiServer
{
public:
    ActionNodeMultiServer() :
        as_countNum_1_(this->nh_, "action_countNum_1", std::bind(&ActionNodeMultiServer::CallbackAsCountNum1, this, std::placeholders::_1), false),
        as_countNum_2_(this->nh_, "action_countNum_2", std::bind(&ActionNodeMultiServer::CallbackAsCountNum2, this, std::placeholders::_1), false),
        as_countNum_3_(this->nh_, "action_countNum_3", std::bind(&ActionNodeMultiServer::CallbackAsCountNum3, this, std::placeholders::_1), false){};

    ~ActionNodeMultiServer(){};

    void StartActionServer()
    {
        // start action server
        this->as_countNum_1_.start();
        this->as_countNum_2_.start();
        this->as_countNum_3_.start();
    }

private:
    // ros node
    ros::NodeHandle nh_;

    // action server
    actionlib::SimpleActionServer<ros_multithread_test::CountNumAction> as_countNum_1_;
    actionlib::SimpleActionServer<ros_multithread_test::CountNumAction> as_countNum_2_;
    actionlib::SimpleActionServer<ros_multithread_test::CountNumAction> as_countNum_3_;

    // action server callback
    void CallbackAsCountNum1(const ros_multithread_test::CountNumGoalConstPtr &goal)
    {
        ros_multithread_test::CountNumFeedback feedback;

        ROS_INFO_STREAM("server1 sleep 10s");
        sleep(5);
        ROS_INFO_STREAM("server1 sleep finished");

        ros::Rate rate(1);
        for (int i = 0; i < goal->count_target; i++)
        {
            ROS_INFO_STREAM("server1 count: " << i);
            feedback.count_current = i;
            this->as_countNum_1_.publishFeedback(feedback);
            rate.sleep();
        }

        ROS_INFO_STREAM("server1 finished");
        this->as_countNum_1_.setSucceeded();
    }

    // action server callback
    void CallbackAsCountNum2(const ros_multithread_test::CountNumGoalConstPtr &goal)
    {
        // ROS_INFO_STREAM("server2 sleep 10s");
        // sleep(10);
        // ROS_INFO_STREAM("server2 sleep finished");

        ros_multithread_test::CountNumFeedback feedback;
        ros::Rate rate(1);
        for (int i = 0; i < goal->count_target; i++)
        {
            ROS_INFO_STREAM("server2 count: " << i);
            feedback.count_current = i;
            this->as_countNum_2_.publishFeedback(feedback);
            rate.sleep();
        }

        ROS_INFO_STREAM("server2 finished");
        this->as_countNum_2_.setSucceeded();
    }

    // action server callback
    void CallbackAsCountNum3(const ros_multithread_test::CountNumGoalConstPtr &goal)
    {
        ros_multithread_test::CountNumFeedback feedback;
        ros::Rate rate(1);
        for (int i = 0; i < goal->count_target; i++)
        {
            ROS_INFO_STREAM("server3 count: " << i);
            feedback.count_current = i;
            this->as_countNum_3_.publishFeedback(feedback);
            rate.sleep();
        }

        ROS_INFO_STREAM("server3 finished");
        this->as_countNum_1_.setSucceeded();
    }
};

int main(int argc, char **argv)
{
    // init ros
    std::string node_name = "action_node_multi_server";
    ros::init(argc, argv, node_name);
    ROS_INFO("%s is running ...", node_name.c_str());

    // create node
    ActionNodeMultiServer action_node_multi_server;

    // start action server
    action_node_multi_server.StartActionServer();

    /**
     * @brief ros::spin()
     * 单纯的spin，无法在主循环中加入额外操作，
     */

    // // ros spin
    // ROS_INFO_STREAM("start spin");
    // ros::spin();

    /**
     * @brief ros::Asyncspinner
     * 多线程模式
     */
    ros::AsyncSpinner spinner(0);
    spinner.start();

    int count = 0;
    ros::Rate rate(1);
    while(ros::ok())
    {
        ROS_INFO_STREAM("main count: " << count);
        count++;
        rate.sleep();
    }

    return 0;
}

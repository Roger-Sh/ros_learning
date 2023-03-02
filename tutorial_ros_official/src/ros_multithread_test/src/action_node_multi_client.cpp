#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include "ros_multithread_test/CountNumAction.h"

class ActionNodeMultiClient
{
public:
    ActionNodeMultiClient() :
        ac_countNum_1_("action_countNum_1", true),
        ac_countNum_2_("action_countNum_2", true),
        ac_countNum_3_("action_countNum_3", true)
    {
        // wait for server
        // blocking the current thread
        // ROS_INFO_STREAM("client wait for action server");
        // this->ac_countNum_1_.waitForServer(ros::Duration(10));
        // ROS_INFO_STREAM("client connected to action server");

        // this->ac_countNum_2_.waitForServer(ros::Duration(10));
        // this->ac_countNum_3_.waitForServer(ros::Duration(10));
    };

    ~ActionNodeMultiClient(){};

    void Run()
    {
        CallClient1();
        // sleep(2);
        CallClient1();
        // sleep(2);
        CallClient2();
    }

private:
    void CallClient1()
    {
        // wait for action server
        // blocking
        ROS_INFO_STREAM("client1 wait for action server");
        this->ac_countNum_1_.waitForServer(ros::Duration(10));
        ROS_INFO_STREAM("client1 connected to action server");

        // set action goal
        ros_multithread_test::CountNumGoal goal;
        goal.count_target = 10;

        // send goal
        this->ac_countNum_1_.sendGoal(
            goal,
            std::bind(&ActionNodeMultiClient::CallbackDoneAcCountNum1, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionNodeMultiClient::CallbackActiveAcCountNum1, this),
            std::bind(&ActionNodeMultiClient::CallbackFeedbackAcCountNum1, this, std::placeholders::_1));
        ROS_INFO_STREAM("client1 sended to action server");

        // // wait for result
        // // blocking
        // if (this->ac_countNum_1_.waitForResult(ros::Duration(20)))
        // {
        //     ROS_INFO_STREAM("Client1 finished");
        // } else {
        //     ROS_INFO_STREAM("Client1 didn't finished within the timeout");
        // }
        
    }

    void CallClient2()
    {
        // wait for action server
        // blocking
        ROS_INFO_STREAM("client2 wait for action server");
        this->ac_countNum_2_.waitForServer(ros::Duration(10));
        ROS_INFO_STREAM("client2 connected to action server");

        // set action goal
        ros_multithread_test::CountNumGoal goal;
        goal.count_target = 10;

        // send goal
        this->ac_countNum_2_.sendGoal(
            goal,
            std::bind(&ActionNodeMultiClient::CallbackDoneAcCountNum2, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionNodeMultiClient::CallbackActiveAcCountNum2, this),
            std::bind(&ActionNodeMultiClient::CallbackFeedbackAcCountNum2, this, std::placeholders::_1));
        ROS_INFO_STREAM("client2 sended to action server");

        // // wait for result
        // // blocking
        // if (this->ac_countNum_2_.waitForResult(ros::Duration(20)))
        // {
        //     ROS_INFO_STREAM("Client2 finished");
        // } else {
        //     ROS_INFO_STREAM("Client2 didn't finished within the timeout");
        // }
    }

    void CallClient3()
    {
        // wait for action server
        // blocking
        ROS_INFO_STREAM("client3 wait for action server");
        this->ac_countNum_3_.waitForServer(ros::Duration(10));
        ROS_INFO_STREAM("client3 connected to action server");

        // set action goal
        ros_multithread_test::CountNumGoal goal;
        goal.count_target = 10;

        // send goal
        this->ac_countNum_3_.sendGoal(
            goal,
            std::bind(&ActionNodeMultiClient::CallbackDoneAcCountNum3, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionNodeMultiClient::CallbackActiveAcCountNum3, this),
            std::bind(&ActionNodeMultiClient::CallbackFeedbackAcCountNum3, this, std::placeholders::_1));
        ROS_INFO_STREAM("client3 sended to action server");

        // // wait for result
        // // blocking
        // if (this->ac_countNum_3_.waitForResult(ros::Duration(20)))
        // {
        //     ROS_INFO_STREAM("Client3 finished");
        // } else {
        //     ROS_INFO_STREAM("Client3 didn't finished within the timeout");
        // }
    }

private:
    // ros node
    ros::NodeHandle nh_;

    // action client
    actionlib::SimpleActionClient<ros_multithread_test::CountNumAction> ac_countNum_1_;
    actionlib::SimpleActionClient<ros_multithread_test::CountNumAction> ac_countNum_2_;
    actionlib::SimpleActionClient<ros_multithread_test::CountNumAction> ac_countNum_3_;

private:
    /**
     * @brief callback of ac_countNum_1_
     *
     */

    void CallbackActiveAcCountNum1()
    {
        ROS_INFO_STREAM("Client1: Goal just went active");
    }

    void CallbackFeedbackAcCountNum1(const ros_multithread_test::CountNumFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("Client1: Feedback: " << feedback->count_current);
    }

    void CallbackDoneAcCountNum1(const actionlib::SimpleClientGoalState& state, const ros_multithread_test::CountNumResultConstPtr& result)
    {
        ROS_INFO_STREAM("Client1: Result: " << result->count_result);
        ROS_INFO_STREAM("Client1: State: " << state.toString());
    }

    /**
     * @brief callback of ac_countNum_2_
     *
     */

    void CallbackActiveAcCountNum2()
    {
        ROS_INFO_STREAM("Client2: Goal just went active");
    }

    void CallbackFeedbackAcCountNum2(const ros_multithread_test::CountNumFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("Client2: Feedback: " << feedback->count_current);
    }

    void CallbackDoneAcCountNum2(const actionlib::SimpleClientGoalState& state, const ros_multithread_test::CountNumResultConstPtr& result)
    {
        ROS_INFO_STREAM("Client2: Result: " << result->count_result);
        ROS_INFO_STREAM("Client2: State: " << state.toString());
    }

    /**
     * @brief callback of ac_countNum_3_
     *
     */

    void CallbackActiveAcCountNum3()
    {
        ROS_INFO_STREAM("Client3: Goal just went active");
    }

    void CallbackFeedbackAcCountNum3(const ros_multithread_test::CountNumFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("Client3: Feedback: " << feedback->count_current);
    }

    void CallbackDoneAcCountNum3(const actionlib::SimpleClientGoalState& state, const ros_multithread_test::CountNumResultConstPtr& result)
    {
        ROS_INFO_STREAM("Client3: Result: " << result->count_result);
        ROS_INFO_STREAM("Client3: State: " << state.toString());
    }
};

int main(int argc, char** argv)
{
    // init ros
    std::string node_name = "action_node_multi_client";
    ros::init(argc, argv, node_name);
    ROS_INFO("%s is running ...", node_name.c_str());

    // start ros asyncspinner
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // create node
    ActionNodeMultiClient action_node_multi_client;
    action_node_multi_client.Run();

    // main loop
    int main_loop_count = 0;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ROS_INFO_STREAM("main loop count: " << main_loop_count);
        main_loop_count++;
        loop_rate.sleep();
    }

    return 0;
}

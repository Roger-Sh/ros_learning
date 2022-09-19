#include <actionlib/client/simple_action_client.h>
#include "actionlib_tutorials/DoDishesAction.h"

// 定义 Client 类型
typedef actionlib::SimpleActionClient<actionlib_tutorials::DoDishesAction> Client;


// 当action激活后会调用该回调函数一次
void activeCB()
{
   ROS_INFO("Goal just went active");
}


// 收到feedback后调用该回调函数
void feedbackCB(const actionlib_tutorials::DoDishesFeedbackConstPtr& feedback)
{
   ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}


// 当action完成后会调用该回调函数一次
void doneCB(const actionlib::SimpleClientGoalState& state,
            const actionlib_tutorials::DoDishesResultConstPtr& result)
{
   ROS_INFO("Yay! The dishes are now clean");
   ros::shutdown();
}



int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "do_dishes_client");

    // 定义一个客户端
    Client client("do_dishes", true);

    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // 创建一个action的goal
    actionlib_tutorials::DoDishesGoal goal;
    goal.dishwasher_id = 1;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCB, &activeCB, &feedbackCB);

    ros::spin();

    return 0;
}
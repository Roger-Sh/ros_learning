#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/PrintNumAction.h>
#include <ros/ros.h>

/**
 * @brief printnum_client main
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "printnum_client");

    // create client
    actionlib::SimpleActionClient<actionlib_tutorials::PrintNumAction> ac_print_num("/actionlib_tutorial/action/print_num", true);
    ac_print_num.waitForServer();
    ROS_INFO_STREAM("client is connected to server");

    // prepare msgs
    actionlib_tutorials::PrintNumGoal goal_msg;
    goal_msg.num = 1;
    goal_msg.print_times = 30;
    actionlib_tutorials::PrintNumResultConstPtr result;
    bool call_flag;

    // seng goal with timeout, check results
    ac_print_num.sendGoal(goal_msg);
    ROS_INFO_STREAM("client sended goal to server");
    call_flag = ac_print_num.waitForResult(ros::Duration(10.0));
    if (call_flag)
    {
        ROS_INFO_STREAM("server finished job");
        result = ac_print_num.getResult();
        ROS_INFO_STREAM("result_msg: " << result->result_msg;);
        ROS_INFO_STREAM("printed_cnt: " << result->printed_cnt;);
    }
    else
    {
        ROS_INFO_STREAM("server didn't finish job before timeout, cancel goal");
        ac_print_num.cancelGoal();

        // wait for cancel result
        ac_print_num.waitForResult(ros::Duration());
        result = ac_print_num.getResult();
        ROS_INFO_STREAM("result_msg: " << result->result_msg;);
        ROS_INFO_STREAM("printed_cnt: " << result->printed_cnt;);
    }

    if (!call_flag)
    {
        // prepare rest goal
        if (goal_msg.print_times > result->printed_cnt)
        {
            goal_msg.print_times = goal_msg.print_times - result->printed_cnt;
        }

        // try again without timeout
        ac_print_num.sendGoal(goal_msg);
        ROS_INFO_STREAM("client sended goal to server");
        call_flag = ac_print_num.waitForResult(ros::Duration());
        if (call_flag)
        {
            ROS_INFO_STREAM("server finished job");
            result = ac_print_num.getResult();
            ROS_INFO_STREAM("result_msg: " << result->result_msg;);
            ROS_INFO_STREAM("printed_cnt: " << result->printed_cnt;);
        }
        else
        {
            ROS_INFO_STREAM("server didn't finish job before timeout, cancel goal");
            ac_print_num.cancelGoal();

            ac_print_num.waitForResult();
            result = ac_print_num.getResult();
            ROS_INFO_STREAM("result_msg: " << result->result_msg;);
            ROS_INFO_STREAM("printed_cnt: " << result->printed_cnt;);
        }
    }

    return 0;
}

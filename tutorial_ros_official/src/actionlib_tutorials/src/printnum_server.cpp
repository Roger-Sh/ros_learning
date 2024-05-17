#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/PrintNumAction.h>
#include <ros/ros.h>

/**
 * @brief class PrintNumAction
 * This class provide an ROS action server, which prints num multiple times,
 * and the action goal can be canceled during the processing.
 */
class PrintNumAction
{
public:
    PrintNumAction(const std::string node_name);
    ~PrintNumAction();

private:
    ros::NodeHandle nh_;
    std::string node_name_;
    actionlib::SimpleActionServer<actionlib_tutorials::PrintNumAction> as_printNum_;
    void CallbackAsPrintNum(const actionlib_tutorials::PrintNumGoalConstPtr& goal);
};

/**
 * @brief Construct a new PrintNumAction object
 *
 */
PrintNumAction::PrintNumAction(const std::string node_name) :
    node_name_(node_name),
    as_printNum_(nh_, "/actionlib_tutorial/action/print_num", boost::bind(&PrintNumAction::CallbackAsPrintNum, this, _1), false)
{
    this->as_printNum_.start();
    ROS_INFO_STREAM("PrintNumAction: as_printNum_ is up.");
}

/**
 * @brief Destroy the Print Num PrintNumAction object
 *
 */
PrintNumAction::~PrintNumAction() {}

/**
 * @brief Callback of action server as_printNum_
 *
 * @param goal
 */
void PrintNumAction::CallbackAsPrintNum(const actionlib_tutorials::PrintNumGoalConstPtr& goal)
{
    ROS_INFO_STREAM("PrintNumAction::CallbackAsPrintNum: goal->num: " << goal->num);
    ROS_INFO_STREAM("PrintNumAction::CallbackAsPrintNum: goal->print_times: " << goal->print_times);
    actionlib_tutorials::PrintNumFeedback feedback;
    actionlib_tutorials::PrintNumResult result;
    bool success = true;

    for (size_t i = 0; i < goal->print_times; i++)
    {
        // check preempted
        if (this->as_printNum_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO_STREAM("PrintNumAction::CallbackAsPrintNum: goal is canceled");
            result.result_msg = result.ResultMsgCanceled;
            result.printed_cnt = i;
            this->as_printNum_.setPreempted(result);
            success = false;
            break;
        }

        // process goal
        ROS_INFO_STREAM("num: " << goal->num << ", time: " << i + 1);
        feedback.printed_cnt = i + 1;
        this->as_printNum_.publishFeedback(feedback);
        ros::Duration(1.0).sleep();
    }

    // check seccess
    if (success)
    {
        result.result_msg = result.ResultMsgFinished;
        result.printed_cnt = goal->print_times;
        this->as_printNum_.setSucceeded(result);
    }
}

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "printnum_server");

    PrintNumAction print_num("printnum_server");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    while (ros::ok())
    {
        ros::Duration(0.1).sleep();
    }

    return 0;
}

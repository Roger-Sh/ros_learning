#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>        // msg FibonacciAction

class FibonacciAction
{
protected:
    // ros node handle
    ros::NodeHandle nh_;

    // action server, 
    // msg type: actionlib_tutorials::FibonacciAction
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; 

    // action name
    std::string action_name_;

    // create messages that are used to published feedback/result
    // msg: action server feedback
    actionlib_tutorials::FibonacciFeedback feedback_;
    // msg: action server result
    actionlib_tutorials::FibonacciResult result_;

public:

    // constructor 构造函数
    // In the action constructor, an action server is created. 
    // The action server takes arguments of a node handle, name of the action, and optionally an executeCB, _1 是占位符. 
    // argfalse: false, A boolean value that tells the ActionServer whether or not to start publishing as soon as it comes up
    // In this example the action server is created with the arguments for the executeCB (callback).
    // 构造函数，需要action name，execute_callback， 并启动execute_callback，当有action client消息传来时，调用execuete_callback
    FibonacciAction(std::string name) : 
        as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
        action_name_(name)
    {
        // 启动action server
        as_.start();
    }

    // destructor 析构函数
    ~FibonacciAction(void)
    {
    }

    // Now the executeCB function referenced in the constructor is created. 
    // The callback function is passed a pointer to the goal message. 
    // Note: This is a boost shared pointer, 
    // given by appending "ConstPtr" to the end of the goal message type.
    // action server 的callback函数，接受goal msg，即目标消息，需要加ConstPtr作为结尾
    void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
    {
        // init some helper variables
        ros::Rate r(1);         // ros loop rate
        bool success = true;    // 是否成功的flag

        // push_back the seeds for the fibonacci sequence
        // 添加fibonacci的前两个基础元素
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // 具体的action 执行阶段, 发布feedback消息
        // start executing the action
        for(int i=1; i<=goal->order; i++)
        {
            // check that preempt has not been requested by the client
            // preempt 抢占，抢先，取代，这里可理解为取消action
            // 如果失败或取消，设置失败及取消状态，退出循环
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }

            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
        
            // publish the feedback
            as_.publishFeedback(feedback_);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }

        // 如果成功，发布结果，设置成功状态
        if(success)
        {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }


};




int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "fibonacci");

    FibonacciAction fibonacci("fibonacci");
    ros::spin();

    return 0;
}
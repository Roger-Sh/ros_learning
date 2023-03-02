#include "ros/ros.h"
#include "ros_multithread_test/AddTwoInts.h"

/**
 * @brief class ServiceNodeMultiServer
 * 测试同一个节点内的多个 server callback
 * server callback 通过 AsyncSpinner 实现多线程响应, 多线程数量设置为0可以根据CPU核心数来分配线程，效率最高
 * 尽量避免server中使用耗时操作, 耗时的操作可以使用action server
 */
class ServiceNodeMultiServer
{
public:
    ServiceNodeMultiServer()
    {
        // init server
        this->server1_ = this->nh_.advertiseService("server1", &ServiceNodeMultiServer::server1_callback, this);
        this->server2_ = this->nh_.advertiseService("server2", &ServiceNodeMultiServer::server2_callback, this);
        this->server3_ = this->nh_.advertiseService("server3", &ServiceNodeMultiServer::server3_callback, this);
    }

    ~ServiceNodeMultiServer(){};

private:
    // ros node handle
    ros::NodeHandle nh_;

    // ros server
    ros::ServiceServer server1_;
    ros::ServiceServer server2_;
    ros::ServiceServer server3_;

private:
    bool server1_callback(ros_multithread_test::AddTwoInts::Request &req, ros_multithread_test::AddTwoInts::Response &res)
    {
        ROS_WARN_STREAM("server1 get req, a: " << req.a << ", b: " << req.b);
        res.sum = req.a + req.b;

        // count loop
        ros::Rate loop_rate(1);
        for(int i = 0; i < 5; i++)
        {
            ROS_INFO_STREAM("server1 count: " << i);
            loop_rate.sleep();
        }

        ROS_WARN_STREAM("server1 send res, sum: " << res.sum);
        return true;
    }

    bool server2_callback(ros_multithread_test::AddTwoInts::Request &req, ros_multithread_test::AddTwoInts::Response &res)
    {
        ROS_WARN_STREAM("server2 get req, a: " << req.a << ", b: " << req.b);
        res.sum = req.a + req.b;

        // count loop
        ros::Rate loop_rate(1);
        for(int i = 0; i < 5; i++)
        {
            ROS_INFO_STREAM("server2 count: " << i);
            loop_rate.sleep();
        }

        ROS_WARN_STREAM("server2 send res, sum: " << res.sum);
        return true;
    }

    bool server3_callback(ros_multithread_test::AddTwoInts::Request &req, ros_multithread_test::AddTwoInts::Response &res)
    {
        ROS_WARN_STREAM("server3 get req, a: " << req.a << ", b: " << req.b);
        res.sum = req.a + req.b;

        // count loop
        ros::Rate loop_rate(1);
        for(int i = 0; i < 5; i++)
        {
            ROS_INFO_STREAM("server3 count: " << i);
            loop_rate.sleep();
        }

        ROS_WARN_STREAM("server3 send res, sum: " << res.sum);
        return true;
    }
};

int main(int argc, char **argv)
{
    // init ros
    std::string node_name = "service_node_multi_server";
    ros::init(argc, argv, node_name);
    ROS_INFO("%s is running ...", node_name.c_str());

    // create node
    ServiceNodeMultiServer node_multi_server;

    /**
     * @brief AsyncSpinner
     * 
     */

    // set thread_count 0 = number of CPU core
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // main loop
    int main_loop_count = 0;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ROS_INFO_STREAM("main loop count: " << main_loop_count);
        loop_rate.sleep();
        main_loop_count++;
    }

    /**
     * @brief spinOnce
     * 
     */

    // // main loop
    // int main_loop_count = 0;
    // ros::Rate loop_rate(1);
    // while (ros::ok())
    // {
    //     ROS_INFO_STREAM("main loop count: " << main_loop_count);
    //     loop_rate.sleep();
    //     main_loop_count++;
    //     ros::spinOnce();
    // }


    return 0;
}

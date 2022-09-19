#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    // init ros
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;

    // get ros param
    std::string topicName;
    nh.param<std::string>("topic_name", topicName, "message");

    // init publisher
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>(topicName, 100);

    // ros loop
    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;

        // 发送的消息中包含namespace
        std::string ns = nh.getNamespace().compare("/") == 0 ? "" : nh.getNamespace();
        ss << ns << "/" << topicName << " " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
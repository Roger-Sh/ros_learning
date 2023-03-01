#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief class TopicNodeMultiPuber
 * pub msgs with different rate
 */
class TopicNodeMultiPuber {
public:
    TopicNodeMultiPuber() {
        // init ros publishers
        this->puber_1_ = this->nh_.advertise<std_msgs::String>("puber_1", 1);
        this->puber_2_ = this->nh_.advertise<std_msgs::String>("puber_2", 1);
        this->puber_3_ = this->nh_.advertise<std_msgs::String>("puber_3", 1);
    };

    ~TopicNodeMultiPuber(){};

    void pub() {
        ros::Rate loop_rate(1);
        int count = 0;

        while (ros::ok()) {
            if (count % 1 == 0) {
                std_msgs::String puber_1_msg;
                std::stringstream ss;
                ss << "puber_1 pub msg, count: " << count;
                puber_1_msg.data = ss.str();
                this->puber_1_.publish(puber_1_msg);
                ROS_INFO(ss.str().c_str());
            }

            if (count % 2 == 0) {
                std_msgs::String puber_2_msg;
                std::stringstream ss;
                ss << "puber_2 pub msg, count: " << count;
                puber_2_msg.data = ss.str();
                this->puber_2_.publish(puber_2_msg);
                ROS_INFO(ss.str().c_str());
            }

            if (count % 5 == 0) {
                std_msgs::String puber_3_msg;
                std::stringstream ss;
                ss << "puber_3 pub msg, count: " << count;
                puber_3_msg.data = ss.str();
                this->puber_3_.publish(puber_3_msg);
                ROS_INFO(ss.str().c_str());
            }

            count++;
            loop_rate.sleep();
        }
    }

private:
    // ros node
    ros::NodeHandle nh_;

    // ros publisher
    ros::Publisher puber_1_;
    ros::Publisher puber_2_;
    ros::Publisher puber_3_;
};

int main(int argc, char **argv) {
    // init ros
    std::string node_name = "topic_node_multi_puber";
    ros::init(argc, argv, node_name);
    ROS_INFO("%s is running ...", node_name.c_str());

    // create node
    TopicNodeMultiPuber node_multi_puber;

    node_multi_puber.pub();

    return 0;
}

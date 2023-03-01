#include <chrono>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief class TopicNodeMultiSuber
 * 测试同一个节点内多个 subscriber callback
 * -   spinOnce模式下，同一节点中如果存在多个subscriber callback，其中一个callback耗时会阻塞其他callback以及main loop中的其他操作。
 * -   AsyncSpinner模式下，同一节点中如果存在多个subscriber callback，可以通过设置多个线程来避免耗时操作的阻塞影响
 *      -   AsyncSpinner的多线程数量只针对callback，main中后续的操作一直不受影响，哪怕多个callback有耗时操作，多线程设为1，主线程也不影响
 *      -   AsyncSpinner 多线程数量设置为0表示根据CPU核心数来选择线程数
 *      -   AsyncSpinner的多线程数量取决于同一时间可能有多少耗时操作，尽量避免在callback中进行耗时操作
 *      -   callback 中如果存在共享资源，则需要加Mutex锁避免资源竞争问题
 */
class TopicNodeMultiSuber {
public:
    TopicNodeMultiSuber() {
        // init ros subscribers
        this->suber_1_ =
            this->nh_.subscribe("puber_1", 1, &TopicNodeMultiSuber::suber_1_callback_, this);
        this->suber_2_ =
            this->nh_.subscribe("puber_2", 1, &TopicNodeMultiSuber::suber_2_callback_, this);
        this->suber_3_ =
            this->nh_.subscribe("puber_3", 1, &TopicNodeMultiSuber::suber_3_callback_, this);
    };

    ~TopicNodeMultiSuber(){};

private:
    // ros node
    ros::NodeHandle nh_;

    // ros subscriber
    ros::Subscriber suber_1_;
    ros::Subscriber suber_2_;
    ros::Subscriber suber_3_;

private:
    void suber_1_callback_(const std_msgs::String::ConstPtr &msg) {
        ROS_INFO_STREAM("suber1 receive: " << msg->data.c_str());

        sleep(10);

        ROS_WARN_STREAM("suber1 finished" << msg->data.c_str());

    }

    void suber_2_callback_(const std_msgs::String::ConstPtr &msg) {
        ROS_INFO_STREAM("suber2 receive: " << msg->data.c_str());

        sleep(10);

        ROS_WARN_STREAM("suber2 finished" << msg->data.c_str());

    }

    void suber_3_callback_(const std_msgs::String::ConstPtr &msg) {
        ROS_INFO_STREAM("suber3 get: " << msg->data.c_str());
    }
};

int main(int argc, char **argv) {
    // init ros
    std::string node_name = "topic_node_multi_suber";
    ros::init(argc, argv, node_name);
    ROS_INFO("%s is running ...", node_name.c_str());

    // create node
    TopicNodeMultiSuber node_multi_suber;


    /**
     * @brief main loop with ros::spinOnce()
     * spinOnce模式下，同一节点中如果存在多个subscriber callback，
     * 其中一个callback耗时会阻塞其他callback以及main loop中的其他操作。
     */

    // // spinOnce with 10HZ
    // int main_loop_count = 0;
    // ros::Rate loop_rate(1);
    // while (ros::ok()) {
    //     ROS_INFO_STREAM("main loop count: " << main_loop_count);

    //     loop_rate.sleep();
    //     ros::spinOnce();
    //     main_loop_count++;
    // }

    /**
     * @brief AsyncSpinner
     * AsyncSpinner 模式下，main loop仍然可以进行
     * 同一节点中如果存在多个subscriber callback，可以通过设置多个线程来避免耗时操作的阻塞影响
     * 多线程数量只针对callback，主线程有单独的线程。哪怕多个callback有耗时操作，多线程设为1，主线程也不影响
     * 多线程数量设置为0表示根据CPU核心数来选择线程数
     * 多线程数量取决于同一时间可能有多少耗时操作，尽量避免在callback中进行耗时操作
     * 如果存在共享资源，则需要加Mutex锁避免资源竞争问题
     */

    // set thread_count = number of CPU core
    ros::AsyncSpinner spinner(0);   
    spinner.start();

    // main loop
    int main_loop_count = 0;
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ROS_INFO_STREAM("main loop count: " << main_loop_count);
        loop_rate.sleep();
        main_loop_count++;
    }

    ros::waitForShutdown();

    return 0;
}

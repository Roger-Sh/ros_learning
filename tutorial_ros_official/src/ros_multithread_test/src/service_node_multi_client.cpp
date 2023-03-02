#include <thread>

#include "ros/ros.h"
#include "ros_multithread_test/AddTwoInts.h"

/**
 * @brief class ServiceNodeMultiClient
 * ROS client 没有 non-blocking 的方式 (或者称 async call)
 * 尽量避免client调用太耗时 (server端不要有太耗时的操作)
 * 如果必须要使用非阻塞式的方式使用client，可以通过std::thread新建一个线程，不过要注意线程数量，防止线程数量爆炸式增长。最好一次只调用一次。
 */
class ServiceNodeMultiClient
{
public:
    ServiceNodeMultiClient()
    {
        // init client
        this->client1_ = this->nh_.serviceClient<ros_multithread_test::AddTwoInts>("server1");
        this->client2_ = this->nh_.serviceClient<ros_multithread_test::AddTwoInts>("server2");
        this->client3_ = this->nh_.serviceClient<ros_multithread_test::AddTwoInts>("server3");
    };

    ~ServiceNodeMultiClient(){};

    /**
     * @brief call same client multi times at once
     * 
     */
    void call_same_client_multi_times()
    {
        int count = 0;
        ros::Rate loop_rate(1);
        while (ros::ok())
        {
            ROS_WARN_STREAM("count: " << count);
            this->service1_msg_.request.a = count;
            this->service1_msg_.request.b = count;
            this->service2_msg_.request.a = count;
            this->service2_msg_.request.b = count;
            this->service3_msg_.request.a = count;
            this->service3_msg_.request.b = count;

            std::thread thread1_call_client1{&ServiceNodeMultiClient::CallClient1, this};
            ROS_WARN_STREAM("thread1_call_client1");
            // non-blocking: number of client call will increase massively
            // thread1_call_client1.detach();

            std::thread thread2_call_client1{&ServiceNodeMultiClient::CallClient1, this};
            ROS_WARN_STREAM("thread2_call_client1");
            // non-blocking: number of client call will increase massively
            // thread2_call_client1.detach();

            // blocking: wait for thread to end
            thread1_call_client1.join();
            thread2_call_client1.join();

            count++;
            loop_rate.sleep();
        }
    }

    /**
     * @brief call different client at once
     * 
     */
    void call_different_client_at_once()
    {
        int count = 0;
        ros::Rate loop_rate(1);
        while (ros::ok())
        {
            ROS_WARN_STREAM("count: " << count);
            this->service1_msg_.request.a = count;
            this->service1_msg_.request.b = count;
            this->service2_msg_.request.a = count;
            this->service2_msg_.request.b = count;
            this->service3_msg_.request.a = count;
            this->service3_msg_.request.b = count;

            std::thread thread_call_client1{&ServiceNodeMultiClient::CallClient1, this};
            ROS_WARN_STREAM("thread_call_client1");
            // sleep(1);

            std::thread thread2_call_client1{&ServiceNodeMultiClient::CallClient1, this};
            ROS_WARN_STREAM("thread_call_client1");
            // non-blocking: number of client call will increase massively
            // thread_call_client1.detach();

            std::thread thread_call_client2{&ServiceNodeMultiClient::CallClient2, this};
            ROS_WARN_STREAM("thread_call_client2");
            // non-blocking: number of client call will increase massively
            // thread_call_client2.detach();

            std::thread thread_call_client3{&ServiceNodeMultiClient::CallClient3, this};
            ROS_WARN_STREAM("thread_call_client3");
            // non-blocking: number of client call will increase massively
            // thread_call_client3.detach();

            // blocking: wait for thread to end
            thread_call_client1.join();
            thread2_call_client1.join();
            thread_call_client2.join();
            thread_call_client3.join();

            count++;
            loop_rate.sleep();
        }
    }

    void CallClient1()
    {
        ROS_WARN_STREAM("client1 call service");

        if (!this->client1_.exists())
        {
            ROS_ERROR("server1 is not available.");
        }

        if (this->client1_.call(this->service1_msg_))
        {
            ROS_WARN("client1 get sum: %ld", (long int)this->service1_msg_.response.sum);
        }
        else
        {
            ROS_ERROR("Failed to call server1");
        }
    }

    void CallClient2()
    {
        // this->client1_.waitForExistence(ros::Duration(10));
        ROS_WARN_STREAM("client2 call service");
        if (this->client2_.call(this->service2_msg_))
        {
            ROS_WARN("client2 get sum: %ld", (long int)this->service2_msg_.response.sum);
        }
        else
        {
            ROS_ERROR("Failed to call server2");
        }
    }

    void CallClient3()
    {
        // this->client1_.waitForExistence(ros::Duration(10));
        ROS_WARN_STREAM("client3 call service");
        if (this->client3_.call(this->service3_msg_))
        {
            ROS_WARN("client3 get sum: %ld", (long int)this->service3_msg_.response.sum);
        }
        else
        {
            ROS_ERROR("Failed to call server3");
        }
    }

private:
    // ros node
    ros::NodeHandle nh_;

    // ros client
    ros::ServiceClient client1_;
    ros::ServiceClient client2_;
    ros::ServiceClient client3_;

    // service msg
    ros_multithread_test::AddTwoInts service1_msg_;
    ros_multithread_test::AddTwoInts service2_msg_;
    ros_multithread_test::AddTwoInts service3_msg_;
};

int main(int argc, char **argv)
{
    // init ros
    std::string node_name = "service_node_multi_client";
    ros::init(argc, argv, node_name);
    ROS_INFO("%s is running ...", node_name.c_str());

    // create node
    ServiceNodeMultiClient node_multi_client;

    // node_multi_client.call_same_client_multi_times();
    node_multi_client.call_different_client_at_once();

    return 0;
}

/*  
    订阅 turtle1 和 turtle2 的 TF 广播信息，查找并转换时间最近的 TF 信息
    将 turtle1 转换成相对 turtle2 的坐标，在计算线速度和角速度并发布

    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.创建 ros 句柄
        4.创建 TF 订阅对象
        5.处理订阅到的 TF
        6.spin

*/
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

#define EPS 1e-3

int main(int argc, char *argv[])
{
    // set local
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"sub_TF");
    ros::NodeHandle nh;

    // tf listener
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // pub: /turtle2/cmd_vel
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1000);

    // ros loop
    ros::Rate rate(10);
    while (ros::ok())
    {
        try
        {
            // get turtle1_in_turtle2 coordinate
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));

            // get vel msg: geometry_msgs/Twist.h
            geometry_msgs::Twist twist;
            twist.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x,2) + pow(tfs.transform.translation.y,2));
            twist.angular.z = 4.0 * atan2(tfs.transform.translation.y,tfs.transform.translation.x);

            // pub vel msg
            if(twist.linear.x > EPS)
            {
                pub.publish(twist);
            }
            else
            {
                ROS_INFO("Turle2 reached the pose of turtle1.");
            }
            
        }
        catch(const std::exception& e)
        {
            ROS_INFO("Exceptiob: %s",e.what());
        }


        // sleep and spin
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
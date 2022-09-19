#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/* 
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建静态坐标转换广播器
        4.创建坐标系信息
        5.广播器发布坐标系信息
        6.spin()
*/

int main(int argc, char *argv[])
{
    // 设置区域相关编码
    setlocale(LC_ALL, "");

    // ros init
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle nh;

    // static transformer broadcaster
    tf2_ros::StaticTransformBroadcaster broadcaster;

    // tf msgs
    geometry_msgs::TransformStamped ts;
    // header
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";       // 父坐标系
    // child frame
    ts.child_frame_id = "laser_link";            // 子坐标系
    // transform of child in parent frame
    ts.transform.translation.x = 0.2;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.5;
    // quaternion
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    // broadcast
    broadcaster.sendTransform(ts);
    ros::spin();
    

    return 0;
}
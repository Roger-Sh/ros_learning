/*

需求:
    现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，
    son1 相对于 world，以及 son2 相对于 world 的关系是已知的，
    求 son1 与 son2中的坐标关系，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标
实现流程:
    1.包含头文件
    2.初始化 ros 节点
    3.创建 ros 句柄
    4.创建 TF 订阅对象
    5.解析订阅信息中获取 son1 坐标系原点在 son2 中的坐标
      解析 son1 中的点相对于 son2 的坐标
    6.spin

*/
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char *argv[])
{   
    // set local
    
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"sub_frames");
    ros::NodeHandle nh;

    // tf listener
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);

    // ros loop
    ros::Rate r(1);
    while (ros::ok())
    {
        try
        {
        //   解析 son1 中的点相对于 son2 的坐标
            geometry_msgs::TransformStamped son1_in_son2 = 
                buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("Son1 in Son2:");
            ROS_INFO("Parent Frame ID = %s",
                son1_in_son2.header.frame_id.c_str());
            ROS_INFO("Child Frame ID = %s",
                son1_in_son2.child_frame_id.c_str());
            ROS_INFO("Son1 in Son2: x=%.2f,y=%.2f,z=%.2f",
                son1_in_son2.transform.translation.x,
                son1_in_son2.transform.translation.y,
                son1_in_son2.transform.translation.z
                );

            // 坐标点解析
            geometry_msgs::PointStamped point1_in_son1;
            point1_in_son1.header.frame_id = "son1";
            point1_in_son1.header.stamp = ros::Time::now();
            point1_in_son1.point.x = 1.0;
            point1_in_son1.point.y = 2.0;
            point1_in_son1.point.z = 3.0;

            geometry_msgs::PointStamped point1_in_son2;
            point1_in_son2 = buffer.transform(point1_in_son1,"son2");
            ROS_INFO("Point1 in Son2:x=%.2f,y=%.2f,z=%.2f",
                point1_in_son2.point.x,
                point1_in_son2.point.y,
                point1_in_son2.point.z
                );
        }
        catch(const std::exception& e)
        {
            ROS_INFO("Exception: %s", e.what());
        }

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
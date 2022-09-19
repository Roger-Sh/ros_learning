/*  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin()
*/

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 

int main(int argc, char *argv[])
{
    // local code format
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"tf_sub");
    ros::NodeHandle nh;

    // tfs listener
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // ros loop
    ros::Rate r(1);
    while (ros::ok())
    {
        // point in laser_link frame
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser_link";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1;
        point_laser.point.y = 2;
        point_laser.point.z = 7.3;


        // point in base_link
        // use try, or it will fail 
        // the first msg will almost always fail
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");
            ROS_INFO("Point after transform: %.2f,%.2f,%.2f",
                point_base.point.x,
                point_base.point.y,
                point_base.point.z);
            ROS_INFO("Parent-frame: %s", point_base.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("exception.....");
        }


        r.sleep();  
        ros::spinOnce();
    }


    return 0;
}
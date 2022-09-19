/*  
    动态的坐标系相对姿态发布(一个坐标系相对于另一个坐标系的相对姿态是不断变动的)

    需求: 启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘
    控制乌龟运动，将两个坐标系的相对位置动态发布

    实现分析:
        1.乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
        2.订阅 turtle1/pose,可以获取乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
        3.将 pose 信息转换成 坐标系相对信息并发布

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建订阅对象
        5.回调函数处理订阅到的数据(实现TF广播)
            5-1.创建 TF 广播器
            5-2.创建 广播的数据(通过 pose 设置)
            5-3.广播器发布数据
        6.spin
*/
// 1.包含头文件
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
// #include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <vector>

// doPose callback func, deal with turtlesim::Pose msg
void doPoseCB(const turtlesim::Pose::ConstPtr& pose)
{
    // tf2 broadcaster
    // note: in CB func, use static to use the broadcaster instance repeatedly
    static tf2_ros::TransformBroadcaster broadcaster;

    // vector of tf_msgs
    std::vector<geometry_msgs::TransformStamped> tf_vector;

    // tf_msg: turtle1 in world
    geometry_msgs::TransformStamped turtle1_in_world;
    turtle1_in_world.header.frame_id = "world";
    turtle1_in_world.header.stamp = ros::Time::now();
    turtle1_in_world.child_frame_id = "turtle1";
    turtle1_in_world.transform.translation.x = pose->x;
    turtle1_in_world.transform.translation.y = pose->y;
    turtle1_in_world.transform.translation.z = 0.0; 
    tf2::Quaternion turtle1_in_world_qtn;
    turtle1_in_world_qtn.setRPY(0,0,pose->theta);
    turtle1_in_world.transform.rotation.x = turtle1_in_world_qtn.getX();
    turtle1_in_world.transform.rotation.y = turtle1_in_world_qtn.getY();
    turtle1_in_world.transform.rotation.z = turtle1_in_world_qtn.getZ();
    turtle1_in_world.transform.rotation.w = turtle1_in_world_qtn.getW();
    tf_vector.push_back(turtle1_in_world);

    // tf_msg: laser in turtle1
    geometry_msgs::TransformStamped laser_in_turtule1;
    laser_in_turtule1.header.frame_id = "turtle1";
    laser_in_turtule1.header.stamp = ros::Time::now();
    laser_in_turtule1.child_frame_id = "laser1";
    laser_in_turtule1.transform.translation.x = 0.5;
    laser_in_turtule1.transform.translation.y = 0.5;
    laser_in_turtule1.transform.translation.z = 1.0; 
    tf2::Quaternion laser_in_turtule1_qtn;
    laser_in_turtule1_qtn.setRPY(0,0,0);
    laser_in_turtule1.transform.rotation.x = laser_in_turtule1_qtn.getX();
    laser_in_turtule1.transform.rotation.y = laser_in_turtule1_qtn.getY();
    laser_in_turtule1.transform.rotation.z = laser_in_turtule1_qtn.getZ();
    laser_in_turtule1.transform.rotation.w = laser_in_turtule1_qtn.getW();
    tf_vector.push_back(laser_in_turtule1);


    // send tf msg
    broadcaster.sendTransform(tf_vector);
}


int main(int argc, char *argv[])
{

    // set local
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"dynamic_tf_pub");
    ros::NodeHandle nh;

    // suber with callback
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, doPoseCB);

    // spin
    ros::spin();
    return 0;
}
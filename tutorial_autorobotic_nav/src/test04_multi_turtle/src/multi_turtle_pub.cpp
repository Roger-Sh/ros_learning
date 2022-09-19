/*  
    该文件实现:需要订阅 turtle1 和 turtle2 的 pose，然后广播相对 world 的坐标系信息

    注意: 订阅的两只 turtle,除了命名空间(turtle1 和 turtle2)不同外,
          其他的话题名称和实现逻辑都是一样的，
          所以我们可以将所需的命名空间通过 args 动态传入

    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.解析传入的命名空间
        4.创建 ros 句柄
        5.创建订阅对象
        6.回调函数处理订阅的 pose 信息
            6-1.创建 TF 广播器
            6-2.将 pose 信息转换成 TransFormStamped
            6-3.发布
        7.spin

*/

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"


//turtle name
std::string turtle_name;


// convert turtlesim::Pose to geometry_msgs::TransformStamped
// pub tf
void doPose(const turtlesim::Pose::ConstPtr& pose){
    // init broadcaster
    // note: in CB func, use static to use the broadcaster instance repeatedly
    static tf2_ros::TransformBroadcaster broadcaster;

    // convert to tf msg
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = turtle_name;
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    // send tf msg
    broadcaster.sendTransform(tfs);
} 




int main(int argc, char *argv[])
{
    // set local
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"pub_tf");
    ros::NodeHandle nh;

    // check args
    if (argc != 2)
    {
        ROS_ERROR("wrong args, please enter turtle name");
    } 
    else 
    {
        turtle_name = argv[1];
        ROS_INFO("Turtle %s start to pub pose.",
            turtle_name.c_str());
    }


    // sub turtle pos
    ros::Subscriber sub = 
        nh.subscribe<turtlesim::Pose>(turtle_name + "/pose", 1000, doPose);

    // ros spin
    ros::spin();
    return 0;
}
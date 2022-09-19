#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件


void pub_laser3_in_world(const geometry_msgs::PoseStamped & laser3_in_world)
{
    // tf2 broadcaster
    static tf2_ros::TransformBroadcaster broadcaster;

    // vector of tf_msgs
    std::vector<geometry_msgs::TransformStamped> tf_vector;

    // tf_msg: turtle1 in world
    geometry_msgs::TransformStamped laser3_in_world_tf;
    laser3_in_world_tf.header.frame_id = "world";
    laser3_in_world_tf.header.stamp = ros::Time::now();
    laser3_in_world_tf.child_frame_id = "laser3";
    laser3_in_world_tf.transform.translation.x = laser3_in_world.pose.position.x;
    laser3_in_world_tf.transform.translation.y = laser3_in_world.pose.position.y;
    laser3_in_world_tf.transform.translation.z = laser3_in_world.pose.position.z; 
    laser3_in_world_tf.transform.rotation.x = laser3_in_world.pose.orientation.x;
    laser3_in_world_tf.transform.rotation.y = laser3_in_world.pose.orientation.y;
    laser3_in_world_tf.transform.rotation.z = laser3_in_world.pose.orientation.z;
    laser3_in_world_tf.transform.rotation.w = laser3_in_world.pose.orientation.w;
    tf_vector.push_back(laser3_in_world_tf);

    // send tf msg
    broadcaster.sendTransform(tf_vector);
}


int main(int argc, char *argv[])
{
    // set local
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"dynamic_tf_sub");
    ros::NodeHandle nh;
    
    // tf listener
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // ros loop
    ros::Rate r(100);
    while (ros::ok())
    {
        // tf: laser in turtle1
        geometry_msgs::PoseStamped laser3_in_turtle1;
        laser3_in_turtle1.header.frame_id = "turtle1";
        laser3_in_turtle1.header.stamp = ros::Time();
        laser3_in_turtle1.pose.position.x = 2.0;
        laser3_in_turtle1.pose.position.y = 2.0;
        laser3_in_turtle1.pose.position.z = 2.0;
        tf2::Quaternion laser3_in_world_qtn;
        laser3_in_world_qtn.setRPY(0,0,0);
        laser3_in_turtle1.pose.orientation.x = laser3_in_world_qtn.getX();
        laser3_in_turtle1.pose.orientation.y = laser3_in_world_qtn.getY();
        laser3_in_turtle1.pose.orientation.z = laser3_in_world_qtn.getZ();
        laser3_in_turtle1.pose.orientation.w = laser3_in_world_qtn.getW();


        // tf: laser in world
        // use try to avoid exception
        try
        {
            // get laser3_in_world
            geometry_msgs::PoseStamped laser3_in_world;
            laser3_in_world = buffer.transform(laser3_in_turtle1, "world");

            // pub laser3_in_world
            pub_laser3_in_world(laser3_in_world);

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("Exception: %s", e.what());
        }




        r.sleep();  
        ros::spinOnce();
    }


    return 0;
}
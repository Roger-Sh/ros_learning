/* 
    创建第二只小乌龟
 */
#include "ros/ros.h"
#include "turtlesim/Spawn.h"


#define PI 3.141592653;


int main(int argc, char *argv[])
{
    // set local
    setlocale(LC_ALL,"");

    // ros init
    ros::init(argc,argv,"create_turtle");
    ros::NodeHandle nh;

    // service client: turtlesim::Spawn
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    ros::service::waitForService("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle2";
    spawn.request.x = 1.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 45.0 /180.0 * PI;
    bool flag = client.call(spawn);

    // check spawn flag
    if (flag)
    {
        ROS_INFO("Turtle %s is generated!",
            spawn.response.name.c_str());
    }
    else
    {
        ROS_INFO("Failed to generate turtle %s.",
            spawn.response.name.c_str());
    }

    ros::spin();

    return 0;
}

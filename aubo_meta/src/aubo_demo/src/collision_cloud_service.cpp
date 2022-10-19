// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// srv
#include <aubo_demo/collisionCloudPub.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief collision object point cloud pub server
 * 
 */
class CollisionCloudPuber
{
public:
    CollisionCloudPuber(const std::string node_name);

    // pub point cloud
    void pc_pub();

private:
    // ros
    std::string node_name_;
    ros::NodeHandle nh_;

    // ros publisher
    ros::Publisher puber_point_cloud_;

    // ros service
    ros::ServiceServer ss_collisionCloudPub_;
    bool ss_collisionCloudPub_CB(
        aubo_demo::collisionCloudPub::Request &req,
        aubo_demo::collisionCloudPub::Response &res);
    aubo_demo::collisionCloudPub::Request collisionCloudPub_req_msg_;
};

/**
 * @brief Construct a new Point Cloud Puber:: Point Cloud Puber object
 * 
 * @param node_name 
 */
CollisionCloudPuber::CollisionCloudPuber(const std::string node_name) :
    node_name_(node_name)
{
    // init ros service server
    this->ss_collisionCloudPub_ = this->nh_.advertiseService(
        "/collisionCloudPub", &CollisionCloudPuber::ss_collisionCloudPub_CB, this);

    // init collisionCloudPub_req_msg_ with prefix value
    this->collisionCloudPub_req_msg_.pub_flag = false;
    this->collisionCloudPub_req_msg_.origin_x = 0.0;
    this->collisionCloudPub_req_msg_.origin_y = 0.0;
    this->collisionCloudPub_req_msg_.origin_z = 0.0;
    this->collisionCloudPub_req_msg_.area_size = 0.1;
    this->collisionCloudPub_req_msg_.resolution = 0.02;
    this->collisionCloudPub_req_msg_.topic = "/collision_object_cloud";

    // init ros puber
    this->puber_point_cloud_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->collisionCloudPub_req_msg_.topic, 1);

}

/**
 * @brief callback of ss_collisionCloudPub_
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool CollisionCloudPuber::ss_collisionCloudPub_CB(
    aubo_demo::collisionCloudPub::Request &req,
    aubo_demo::collisionCloudPub::Response &res)
{
    // update collisionCloudPub_req_msg_
    this->collisionCloudPub_req_msg_ = req;

    // redefine ros puber
    this->puber_point_cloud_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->collisionCloudPub_req_msg_.topic, 1);

    // set service response
    res.msg = "call collisionCloudPub successfully";
    res.ret = 1;

    return true;
}

/**
 * @brief pub point cloud
 * 
 */
void CollisionCloudPuber::pc_pub()
{
    // check pub flag
    if (!this->collisionCloudPub_req_msg_.pub_flag)
    {
        return;
    }

    // create collision object cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    float iter_size = this->collisionCloudPub_req_msg_.area_size / this->collisionCloudPub_req_msg_.resolution;
    for (size_t i = 0; i <= iter_size; i++)
    {
        for (size_t j = 0; j <= iter_size; j++)
        {
            for (size_t k = 0; k <= iter_size; k++)
            {
                pcl::PointXYZ point;
                point.x = this->collisionCloudPub_req_msg_.origin_x 
                    - this->collisionCloudPub_req_msg_.area_size/2.0 
                    + i * this->collisionCloudPub_req_msg_.resolution;
                point.y = this->collisionCloudPub_req_msg_.origin_y 
                    - this->collisionCloudPub_req_msg_.area_size/2.0 
                    + j * this->collisionCloudPub_req_msg_.resolution;
                point.z = this->collisionCloudPub_req_msg_.origin_z 
                    - this->collisionCloudPub_req_msg_.area_size/2.0 
                    + k * this->collisionCloudPub_req_msg_.resolution;

                cloud.push_back(point);
            }
        }
    }
    
    // convert pcl cloud to ros msg
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = this->collisionCloudPub_req_msg_.reference_frame;

    // pub
    this->puber_point_cloud_.publish(cloud_msg);
}

int main(int argc, char **argv)
{
    // init ros
    std::string node_name = "point_cloud_puber";
    ros::init(argc, argv, node_name);

    // create CollisionCloudPuber
    CollisionCloudPuber cc_puber(node_name);

    // ros loop
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // pub collision object point cloud
        cc_puber.pc_pub();

        // spin once
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

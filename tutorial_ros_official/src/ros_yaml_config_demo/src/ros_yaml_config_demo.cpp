#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_yaml_config_demo");

    ros::NodeHandle nh;

    std::vector<std::string> param_names;
    nh.getParamNames(param_names);

    for (size_t i = 0; i < param_names.size(); i++)
    {
        ROS_INFO_STREAM(param_names[i]);
    }
    
    std::string test_param_string;
    int test_param_int;
    double test_param_double;
    std::vector<double> test_param_vec;

    ros::param::get("~test_param_string", test_param_string);
    ros::param::get("~test_param_int", test_param_int);
    ros::param::get("~test_param_double", test_param_double);
    ros::param::get("~test_param_vec", test_param_vec);

    ROS_INFO_STREAM("test_param_string: " << test_param_string);
    ROS_INFO_STREAM("test_param_int: " << test_param_int);
    ROS_INFO_STREAM("test_param_double: " << test_param_double);
    ROS_INFO_STREAM("test_param_vec: ");
    for (size_t i = 0; i < test_param_vec.size(); i++)
    {
        ROS_INFO_STREAM(test_param_vec[i]);
    }

    return 0;
}

#include <ros/ros.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <dynamic_param_demo/dynamic_param_demoConfig.h>

std::string s;
int num;
bool bool_param;
double double_param;
int enum_param;

/**
 * @brief paramConfigCallback, use rqt_reconfigure to reconfigure parameter
 * 
 * @param config 
 * @param level 
 */
void paramConfigCallback(dynamic_param_demo::dynamic_param_demoConfig &config, uint32_t level)
{
	s = config.s;
	num = config.num;
    bool_param = config.bool_param;
    double_param = config.double_param;
    enum_param = config.size;

	printf("\nstring_param:  %s\n", s.c_str());
    printf("int_param:  %d\n", num);
    printf("bool_param:  %d\n", bool_param);
    printf("double_param:  %lf\n", double_param);
    printf("enum_param:  %d\n", enum_param);
}

/**
 * @brief main
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
    // init ros
    ros::init(argc, argv, "dynamic_param_demo");
    ros::NodeHandle pnh("~");
    
    // init dynamic reconfigure server and callback
    dynamic_reconfigure::Server<dynamic_param_demo::dynamic_param_demoConfig> dyna_reconf_server;
  	dynamic_reconfigure::Server<dynamic_param_demo::dynamic_param_demoConfig>::CallbackType dyna_reconf_server_CB;
  	dyna_reconf_server_CB = boost::bind(&paramConfigCallback, _1, _2);
    dyna_reconf_server.setCallback(dyna_reconf_server_CB);
  
    // get ros param 
    pnh.param<std::string>("string_param", s, "default_string");
    pnh.param<int>("int_param", num, 2333);
    pnh.param<bool>("bool_param", bool_param, false);
    pnh.param<double>("double_param", double_param, 1.5);
    pnh.param<int>("enum_param", enum_param, 0);

    // print init param
    printf("\nstring_param:  %s\n", s.c_str());
    printf("int_param:  %d\n", num);
    printf("bool_param:  %d\n", bool_param);
    printf("double_param:  %lf\n", double_param);
    printf("enum_param:  %d\n", enum_param);

    ros::spin();  
}


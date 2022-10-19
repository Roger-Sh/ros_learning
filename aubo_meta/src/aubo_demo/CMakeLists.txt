cmake_minimum_required(VERSION 2.8.3)
project(aubo_demo)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_msgs
    pcl_conversions
    rosbag
    tf
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    aubo_msgs
    message_generation
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# # Generate services in the 'srv' folder
add_service_files(
    FILES
    collisionCloudPub.srv
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(

    # INCLUDE_DIRS include

    # LIBRARIES aubo_demo
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_msgs
    pcl_conversions
    rosbag
    tf
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    aubo_msgs
    message_runtime
    message_generation
    # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########
include_directories(

    # include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)
add_definitions(${EIGEN_DEFINITIONS})

# MoveGroupInterface_To_Melodic
add_executable(MoveGroupInterface_To_Melodic src/MoveGroupInterface_To_Melodic.cpp)
target_link_libraries(MoveGroupInterface_To_Melodic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# aubo_i5_demo
add_executable(aubo_i5_demo src/aubo_i5_demo.cpp)
target_link_libraries(aubo_i5_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# aubo_i5_catch_demo
add_executable(aubo_i5_catch_demo src/aubo_i5_catch_demo.cpp)
target_link_libraries(aubo_i5_catch_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(aubo_i5_catch_demo ${aubo_demo_EXPORTED_TARGETS})


# point_cloud_puber.cpp
add_executable(collision_cloud_service src/collision_cloud_service.cpp)
target_link_libraries(collision_cloud_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(collision_cloud_service ${aubo_demo_EXPORTED_TARGETS})

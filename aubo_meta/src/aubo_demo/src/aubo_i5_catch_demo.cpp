// ros moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// ros tf
#include <tf/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

// aubo driver
#include "aubo_msgs/SetIO.h"

// collsionCloudPub
#include "aubo_demo/collisionCloudPub.h"

// define
#define PLANNING_GROUP_NAME "manipulator_i5"
#define BASE_LINK_NAME "base_link"
#define END_EFFECTOR_LINK_NAME "gripper_tip_Link"
#define ROBOT_DESCRIPTION_NAME "robot_description"

namespace rvt = rviz_visual_tools;
typedef std::map<std::string, moveit_msgs::CollisionObject> CollisionObjectMap;

/**
 * @brief aubo i5 arm catch demo
 *
 */
class AuboI5CatchDemo
{
public:
    AuboI5CatchDemo(
        const std::string node_name,
        const std::string planning_group_name,
        const std::string base_link_name,
        const std::string robot_description_name);

private:
    // ros node
    ros::NodeHandle nh_;
    std::string node_name_;

    // ros service client
    ros::ServiceClient sc_aubo_io_;
    ros::ServiceClient sc_CollisionCloudPub_;


    // moveit::MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

    // moveit::PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // planning_scene_monitor
    planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor_;

    // MoveItVisualTools
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    std::string visual_txt_;

    // robot_state::JointModelGroup
    const robot_state::JointModelGroup *joint_model_group_;

    // aubo arm related
    std::string planning_group_name_;
    std::string base_link_name_;
    std::string robot_description_name_;

private:
    // execute target pose
    bool execute_target_pose(
        const geometry_msgs::Pose &target_pose,
        const std::string pose_name);

    // execute target joint
    bool execute_target_joint(
        const std::vector<double> &target_joint,
        const std::string joint_name);

    // set aubo io
    bool set_aubo_io(int pin, int state);
    bool open_gripper();
    bool close_gripper();
    bool recover_gripper();

    // pub visual text
    void pub_visual_text(const std::string txt);
    std::string visual_tools_info_;

    // add collision object
    void add_collision_object(
        const geometry_msgs::Pose &object_pose,
        const std::string object_name,
        const float object_size);

    // call collisionCloudPub service
    void call_collisionCloudPub_service(    
        const geometry_msgs::Pose &object_pose, 
        const float cloud_size, 
        const float cloud_resolution, 
        const bool pub_flag);
    

    // remove collision object
    void remove_collision_object(const std::string object_name);

    // add acm (allowedCollisionMatrix) entry
    void add_acm_entry_to_scene(
        const std::string allowed_collision_link_1, 
        const std::string allowed_collision_link_2);

    // recover acm (allowedCollisionMatrix)
    void recover_acm();
    collision_detection::AllowedCollisionMatrix acm_origin_;

    // check planning scene
    void check_planning_scene();

public:
    /**
     * @brief move plan
     * 
     */

    void moveplan_go_home();
    void moveplan_catch_demo();
};

/**
 * @brief Construct a new AuboI5CatchDemo::AuboI5CatchDemo object
 *
 * @param node_name
 */
AuboI5CatchDemo::AuboI5CatchDemo(
    const std::string node_name,
    const std::string planning_group_name,
    const std::string base_link_name,
    const std::string robot_description_name ): 
    node_name_(node_name),
    planning_group_name_(planning_group_name),
    base_link_name_(base_link_name),
    robot_description_name_(robot_description_name),
    move_group_interface_(planning_group_name),
    planning_scene_monitor_(robot_description_name),
    visual_tools_(base_link_name)
{
    // init ros service client
    this->sc_aubo_io_ = this->nh_.serviceClient<aubo_msgs::SetIO>("/aubo_driver/set_io");
    this->sc_CollisionCloudPub_ = this->nh_.serviceClient<aubo_demo::collisionCloudPub>("/collisionCloudPub");

    // init move_group_interface_, set frames
    ROS_WARN("End effector link: %s", this->move_group_interface_.getEndEffectorLink().c_str());
    ROS_WARN("PoseReferenceFrame link: %s", this->move_group_interface_.getPoseReferenceFrame().c_str());
    this->move_group_interface_.setEndEffectorLink(END_EFFECTOR_LINK_NAME);
    this->move_group_interface_.setPoseReferenceFrame(this->base_link_name_);

    // init joint_model_group
    this->joint_model_group_ = this->move_group_interface_.getCurrentState()->getJointModelGroup(this->planning_group_name_);

    // init MoveItVisualTools
    this->visual_tools_.deleteAllMarkers();
    this->visual_tools_.loadRemoteControl();
    this->visual_txt_ = "aubo_i5_catch_demo start";
    pub_visual_text(visual_txt_);
}

/**
 * @brief pub text with MoveItVisualTools
 *
 * @param txt
 */
void AuboI5CatchDemo::pub_visual_text(const std::string txt)
{
    // set text pose
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.5;

    // set text
    this->visual_tools_.publishText(text_pose, txt, rvt::WHITE, rvt::XXLARGE);

    // pub visual text marker
    this->visual_tools_.trigger();
}

/**
 * @brief moveplan_go_home
 *
 */
void AuboI5CatchDemo::moveplan_go_home()
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: go home";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    std::vector<double> home_pose;
    home_pose.push_back(0.0 / 180.0 * M_PI);
    home_pose.push_back(-78.0 / 180.0 * M_PI);
    home_pose.push_back(-100.0 / 180.0 * M_PI);
    home_pose.push_back(8.0 / 180.0 * M_PI);
    home_pose.push_back(-90.0 / 180.0 * M_PI);
    home_pose.push_back(88.0 / 180.0 * M_PI);
    this->move_group_interface_.setJointValueTarget(home_pose);
    this->move_group_interface_.move();
}

/**
 * @brief moveplan_catch_demo
 *
 */
void AuboI5CatchDemo::moveplan_catch_demo()
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: start catch";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    /**
     * @brief define pose
     *
     */

    // catch_pose
    geometry_msgs::Pose catch_pose;
    Eigen::Isometry3d catch_pose_m;
    Eigen::Vector3d catch_pose_t(-0.5, -0.252, 0.267);              // x y z
    Eigen::Quaterniond catch_pose_q(-0.019, -0.135, 0.991, -0.005); // w x y z
    catch_pose_m.setIdentity();
    catch_pose_m.translate(catch_pose_t);
    catch_pose_m.rotate(catch_pose_q);
    ROS_WARN_STREAM("catch_pose_m: \n"
                    << catch_pose_m.matrix());
    catch_pose = tf2::toMsg(catch_pose_m);

    // pre_catch_pose
    geometry_msgs::Pose pre_catch_pose;
    Eigen::Isometry3d pre_catch_pose_m;
    pre_catch_pose_m.setIdentity();
    Eigen::Vector3d pre_catch_pose_t(0.0, 0.0, -0.1); // x y z
    pre_catch_pose_m.translate(pre_catch_pose_t);
    pre_catch_pose_m = catch_pose_m * pre_catch_pose_m;
    ROS_WARN_STREAM("pre_catch_pose_m: \n"
                    << pre_catch_pose_m.matrix());
    pre_catch_pose = tf2::toMsg(pre_catch_pose_m);

    // collect_pose
    std::vector<double> collect_pose;
    collect_pose.push_back(-90.0 / 180.0 * M_PI);
    collect_pose.push_back(-0.0 / 180.0 * M_PI);
    collect_pose.push_back(-125.0 / 180.0 * M_PI);
    collect_pose.push_back(-35.0 / 180.0 * M_PI);
    collect_pose.push_back(-90.0 / 180.0 * M_PI);
    collect_pose.push_back(0.0 / 180.0 * M_PI);

    /**
     * @brief add path constraints
     *
     */

    moveit_msgs::Constraints path_constraints;

    // upperArm_joint [-75, 75]
    // moveit_msgs::JointConstraint joint_constraint_upperArm;
    // joint_constraint_upperArm.joint_name = "upperArm_joint";
    // joint_constraint_upperArm.position = 0.0 /180.0*M_PI;
    // joint_constraint_upperArm.tolerance_above = 75.0 /180.0*M_PI;
    // joint_constraint_upperArm.tolerance_below = -75.0 /180.0*M_PI;
    // joint_constraint_upperArm.weight = 1.0;
    // path_constraints.joint_constraints.push_back(joint_constraint_upperArm);

    // foreArm_joint [-150, 0]
    moveit_msgs::JointConstraint joint_constraint_foreArm;
    joint_constraint_foreArm.joint_name = "foreArm_joint";
    joint_constraint_foreArm.position = -45.0 /180.0*M_PI;
    joint_constraint_foreArm.tolerance_above = 0.0 /180.0*M_PI;
    joint_constraint_foreArm.tolerance_below = -170.0 /180.0*M_PI;
    joint_constraint_foreArm.weight = 1.0;
    path_constraints.joint_constraints.push_back(joint_constraint_foreArm);

    // set path constraints
    this->move_group_interface_.setPathConstraints(path_constraints);

    /**
     * @brief execure catch moveplan
     *
     */

    // add catch_object
    add_collision_object(catch_pose, "catch_object", 0.1);

    // start pub collision object cloud
    call_collisionCloudPub_service(catch_pose, 0.1, 0.02, true);

    // update acm
    add_acm_entry_to_scene("gripper_Link", "catch_object");

    // pre_catch_pose
    execute_target_pose(pre_catch_pose, "pre_catch_pose");

    // open gripper
    open_gripper();

    // catch_pose
    execute_target_pose(catch_pose, "catch_pose");

    // close gripper
    close_gripper();

    // attach catch_object
    this->visual_tools_info_ = "Press RVIZ 'next' button: attach catch_object";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);
    this->move_group_interface_.attachObject("catch_object", END_EFFECTOR_LINK_NAME);

    // pre_catch_pose
    execute_target_pose(pre_catch_pose, "pre_catch_pose");

    // stop pub collision object cloud
    call_collisionCloudPub_service(catch_pose, 0.1, 0.02, false);

    // clear path constraints
    this->move_group_interface_.clearPathConstraints();

    // collect_pose
    execute_target_joint(collect_pose, "collect_pose");

    // open gripper
    open_gripper();

    // detach catch_object
    this->visual_tools_info_ = "Press RVIZ 'next' button: detach catch_object";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);
    this->move_group_interface_.detachObject("catch_object");

    // go home
    moveplan_go_home();

    // recover acm
    recover_acm();

    // recover gripper
    recover_gripper();

    // remove catch_object
    this->visual_tools_info_ = "Press RVIZ 'next' button: remove catched object";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);
    remove_collision_object("catch_object");
}

/**
 * @brief execute target pose
 *
 * @return true
 * @return false
 */
bool AuboI5CatchDemo::execute_target_pose(
    const geometry_msgs::Pose &target_pose,
    const std::string pose_name)
{
    // plan target_pose
    this->move_group_interface_.setStartStateToCurrentState();
    this->move_group_interface_.setPoseTarget(target_pose);

    ROS_WARN_STREAM("target_pose \n"
                    << target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan target_pose_plan;
    bool plan_flag = (this->move_group_interface_.plan(target_pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!plan_flag)
    {
        ROS_WARN("%s: bad target pose, fail to plan trajectory.", this->node_name_.c_str());
        return false;
    }

    // visual planning path in Rviz
    this->visual_tools_.deleteAllMarkers();
    this->visual_tools_.publishAxisLabeled(target_pose, pose_name, rvt::LARGE);
    this->visual_tools_.publishTrajectoryLine(target_pose_plan.trajectory_, this->joint_model_group_);
    this->visual_tools_.trigger();
    this->visual_tools_info_ = "Press RVIZ 'next' button: go to ";
    this->visual_tools_info_.append(pose_name);
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    // execute target_pose_1_plan
    this->move_group_interface_.execute(target_pose_plan);

    return true;
}

/**
 * @brief execute target joint
 *
 * @param target_joint
 * @param joint_name
 * @return true
 * @return false
 */
bool AuboI5CatchDemo::execute_target_joint(
    const std::vector<double> &target_joint,
    const std::string joint_name)
{
    this->move_group_interface_.setJointValueTarget(target_joint);
    moveit::planning_interface::MoveGroupInterface::Plan target_joint_plan;
    bool plan_flag = (this->move_group_interface_.plan(target_joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!plan_flag)
    {
        ROS_WARN("%s: bad target joint, fail to plan trajectory.", this->node_name_.c_str());
        return false;
    }

    // visual planning path in Rviz
    this->visual_tools_.deleteAllMarkers();
    // this->visual_tools_.publishAxisLabeled(target_pose, joint_name, rvt::LARGE);
    this->visual_tools_.publishTrajectoryLine(target_joint_plan.trajectory_, this->joint_model_group_);
    this->visual_tools_.trigger();
    this->visual_tools_info_ = "Press RVIZ 'next' button: go to ";
    this->visual_tools_info_.append(joint_name);
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    this->move_group_interface_.move();
}

/**
 * @brief set aubo io
 *
 * @param pin
 * @param state
 * @return true
 * @return false
 */
bool AuboI5CatchDemo::set_aubo_io(int pin, int state)
{
    aubo_msgs::SetIO io_srv_msg;

    io_srv_msg.request.fun = io_srv_msg.request.FUN_SET_RobotBoardUserDO;
    io_srv_msg.request.pin = pin;
    io_srv_msg.request.state = state;

    if (this->sc_aubo_io_.call(io_srv_msg))
    {
        ROS_INFO("set io: %ld", (long int)io_srv_msg.response.success);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to set io pin: %d, state: %d", pin, state);
        return false;
    }
}

/**
 * @brief set aubo io to open gripper
 *
 * @return true
 * @return false
 */
bool AuboI5CatchDemo::open_gripper()
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: open gripper";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    // reset
    recover_gripper();

    // open gripper
    set_aubo_io(1, 1);
}

/**
 * @brief set aubo io to close gripper
 *
 * @return true
 * @return false
 */
bool AuboI5CatchDemo::close_gripper()
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: close gripper";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    // reset
    recover_gripper();

    // open gripper
    set_aubo_io(0, 1);
}

/**
 * @brief recover gripper 
 * 
 * @return true 
 * @return false 
 */
bool AuboI5CatchDemo::recover_gripper()
{
    // reset gripper
    set_aubo_io(0, 0);
    set_aubo_io(1, 0);
}

/**
 * @brief add collision object with pose and name
 *
 */
void AuboI5CatchDemo::add_collision_object(
    const geometry_msgs::Pose &object_pose,
    const std::string object_name,
    const float object_size)
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: add catch object";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    moveit_msgs::CollisionObject catch_object;
    catch_object.header.frame_id = this->move_group_interface_.getPoseReferenceFrame();
    catch_object.id = object_name;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = object_size;
    primitive.dimensions[1] = object_size;
    primitive.dimensions[2] = object_size;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = object_pose.position.x;
    box_pose.position.y = object_pose.position.y;
    box_pose.position.z = object_pose.position.z;
    catch_object.primitives.push_back(primitive);
    catch_object.primitive_poses.push_back(box_pose);
    catch_object.operation = catch_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(catch_object);
    this->planning_scene_interface_.applyCollisionObjects(collision_objects); // synchronously

    ros::Duration(2.0).sleep();
}

// call collisionCloudPub service
void AuboI5CatchDemo::call_collisionCloudPub_service(
    const geometry_msgs::Pose &object_pose, 
    const float cloud_size, 
    const float cloud_resolution, 
    const bool pub_flag)
{
    // prepare service msg
    aubo_demo::collisionCloudPub collisionCloudPub_msg;
    collisionCloudPub_msg.request.topic = "/collision_object_cloud";
    collisionCloudPub_msg.request.reference_frame = "base_link";
    collisionCloudPub_msg.request.pub_flag = pub_flag;
    collisionCloudPub_msg.request.area_size = cloud_size;
    collisionCloudPub_msg.request.resolution = cloud_resolution;
    collisionCloudPub_msg.request.origin_x = object_pose.position.x;
    collisionCloudPub_msg.request.origin_y = object_pose.position.y;
    collisionCloudPub_msg.request.origin_z = object_pose.position.z;

    // call service
    this->sc_CollisionCloudPub_.call(collisionCloudPub_msg);
}

/**
 * @brief remove collision object with object name
 * 
 * @param object_name 
 */
void AuboI5CatchDemo::remove_collision_object(
    const std::string object_name)
{
    // OBJ to be removed
    moveit_msgs::CollisionObject obj;
    obj.id = object_name;
    obj.operation = obj.REMOVE;

    // apply collision objects to be removed
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(obj);
    this->planning_scene_interface_.applyCollisionObjects(collision_objects); // synchronously
}

/**
 * @brief add acm (allowedCollisionMatrix entry to planning scene msg)
 * 
 * @param allowed_collision_link_1 
 * @param allowed_collision_link_2 
 */
void AuboI5CatchDemo::add_acm_entry_to_scene(
    const std::string allowed_collision_link_1, 
    const std::string allowed_collision_link_2)
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: modify ACM";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    // get current planning scene
    this->planning_scene_monitor_.requestPlanningSceneState("/get_planning_scene");
    planning_scene::PlanningScenePtr scene = this->planning_scene_monitor_.getPlanningScene();

    // get current scene msg
    moveit_msgs::PlanningScene scene_msg;
    scene->getPlanningSceneMsg(scene_msg);

    // get current acm
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
    this->acm_origin_ = acm;

    // // check acm_names
    // std::vector<std::string> acm_names;
    // acm.getAllEntryNames(acm_names);
    // ROS_WARN_STREAM("ACM before modification: ");
    // for (size_t i = 0; i < acm_names.size(); i++)
    // {
    //     ROS_WARN_STREAM(acm_names[i]);
    // }

    // set acm
    acm.setEntry(allowed_collision_link_1, allowed_collision_link_2, true);
    // ROS_WARN_STREAM("ACM after modification: ");
    // acm.getAllEntryNames(acm_names);
    // for (size_t i = 0; i < acm_names.size(); i++)
    // {
    //     ROS_WARN_STREAM(acm_names[i]);
    // }

    // set acm to scene msg
    acm.getMessage(scene_msg.allowed_collision_matrix);
    scene_msg.is_diff = true;

    // apply scene_msg
    this->planning_scene_interface_.applyPlanningScene(scene_msg);

}

/**
 * @brief recover allowedCollisionMatrix to its original config
 * 
 */
void AuboI5CatchDemo::recover_acm()
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: recover ACM";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    // get current planning scene
    this->planning_scene_monitor_.requestPlanningSceneState("/get_planning_scene");
    planning_scene::PlanningScenePtr scene = this->planning_scene_monitor_.getPlanningScene();

    // get current scene msg
    moveit_msgs::PlanningScene scene_msg;
    scene->getPlanningSceneMsg(scene_msg);

    // set acm_origin to scene msg
    this->acm_origin_.getMessage(scene_msg.allowed_collision_matrix);

    // apply scene msg
    this->planning_scene_interface_.applyPlanningScene(scene_msg);

    // check acm after recovery
    // this->planning_scene_monitor_.requestPlanningSceneState("/get_planning_scene");
    // collision_detection::AllowedCollisionMatrix acm = this->planning_scene_monitor_.getPlanningScene()->getAllowedCollisionMatrix();
    // std::vector<std::string> acm_names;
    // acm.getAllEntryNames(acm_names);
    // ROS_WARN_STREAM("ACM after recovery: ");
    // for (size_t i = 0; i < acm_names.size(); i++)
    // {
    //     ROS_WARN_STREAM(acm_names[i]);
    // }
}

// check planning scene
void AuboI5CatchDemo::check_planning_scene()
{
    this->visual_tools_info_ = "Press RVIZ 'next' button: check scene";
    pub_visual_text(this->visual_tools_info_);
    this->visual_tools_.prompt(this->visual_tools_info_);

    // get current planning scene
    this->planning_scene_monitor_.requestPlanningSceneState("/get_planning_scene");
    planning_scene::PlanningScenePtr scene = this->planning_scene_monitor_.getPlanningScene();

    // get scene msg
    moveit_msgs::PlanningScene scene_msg;
    scene->getPlanningSceneMsg(scene_msg);

    ROS_WARN_STREAM("octomap binary: " << scene_msg.world.octomap.octomap.binary);
    ROS_WARN_STREAM("octomap data_size: " << scene_msg.world.octomap.octomap.data.size());
    ROS_WARN_STREAM("octomap id: " << scene_msg.world.octomap.octomap.id);
    ROS_WARN_STREAM("octomap resolution" << scene_msg.world.octomap.octomap.resolution);



}

/**
 * @brief main
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // init ros
    std::string node_name = "aubo_i5_catch_demo";
    ros::init(argc, argv, node_name);

    // Start a thread for subscriber, could use more thread for multi subscriber
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // create AuboI5CatchDemo
    AuboI5CatchDemo aubo_catch_demo(
        node_name, 
        PLANNING_GROUP_NAME, 
        BASE_LINK_NAME, 
        ROBOT_DESCRIPTION_NAME);

    // execute move plan
    aubo_catch_demo.moveplan_go_home();

    aubo_catch_demo.moveplan_catch_demo();

    return 0;
}    // check planning scene
    void check_planning_scene();
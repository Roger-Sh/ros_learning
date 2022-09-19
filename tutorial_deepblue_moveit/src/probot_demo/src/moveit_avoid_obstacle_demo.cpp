#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>


#include <vector>
#include <iostream>

/**
 * @brief joint move, joint target
 * 
 * @param arm 
 */
void moveit_jointmove_joint_target(moveit::planning_interface::MoveGroupInterface &arm)
{
/**
     * @brief move
     * 
     */

    // 控制机械臂先回到初始化位置
    ROS_INFO("start home");
    arm.setNamedTarget("home");
    arm.move();
    ROS_INFO("finish home");
    sleep(1);

    double rad_fac = 3.14159/180.0;

    while(true)
    {
        // 设置关节运动目标
        double targetPose1[6] = {
            145.0 * rad_fac, 
            5.0 * rad_fac, 
            170.0 * rad_fac, 
            -5.0 * rad_fac, 
            184.0 * rad_fac, 
            -165.0 * rad_fac};
        std::vector<double> joint_group_positions_1(6);
        joint_group_positions_1[0] = targetPose1[0];
        joint_group_positions_1[1] = targetPose1[1];
        joint_group_positions_1[2] = targetPose1[2];
        joint_group_positions_1[3] = targetPose1[3];
        joint_group_positions_1[4] = targetPose1[4];
        joint_group_positions_1[5] = targetPose1[5];
        arm.setJointValueTarget(joint_group_positions_1);

        // 执行关节运动
        ROS_INFO("start joint target 1");
        arm.move();
        ROS_INFO("finish joint target 1");
        sleep(1);

        // 设置关节运动目标
        double targetPose2[6] = {
            -141.0 * rad_fac, 
            -15.0 * rad_fac, 
            0.0 * rad_fac, 
            0.0 * rad_fac, 
            20.0 * rad_fac, 
            -42.0 * rad_fac};
        std::vector<double> joint_group_positions_2(6);
        joint_group_positions_2[0] = targetPose2[0];
        joint_group_positions_2[1] = targetPose2[1];
        joint_group_positions_2[2] = targetPose2[2];
        joint_group_positions_2[3] = targetPose2[3];
        joint_group_positions_2[4] = targetPose2[4];
        joint_group_positions_2[5] = targetPose2[5];
        arm.setJointValueTarget(joint_group_positions_2);

        // 执行关节运动
        ROS_INFO("start joint target 2");
        arm.move();
        ROS_INFO("finish joint target 2");
        sleep(1);
    }
}

/**
 * @brief cartesian move, cartesian pose target
 * 
 * @param arm 
 */
void moveit_cartmove_pose_target(
    moveit::planning_interface::MoveGroupInterface &arm,
    std::string end_effector_link
)
{
    /**
     * @brief home pose, joint move, joint target
     * 
     */

    // 控制机械臂先回到初始化位置
    ROS_INFO("start home");
    arm.setNamedTarget("home");
    arm.move();
    ROS_INFO("finish home");
    sleep(1);


    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    // pose
    geometry_msgs::Pose start_pose;
    start_pose.orientation.x = 0.611;
    start_pose.orientation.y = -0.303;
    start_pose.orientation.z = -0.301;
    start_pose.orientation.w = 0.667;
    start_pose.position.x = 0.199;
    start_pose.position.y = -0.143;
    start_pose.position.z = 0.421;

    // geometry_msgs::Pose way_pose_1;
    // way_pose_1.orientation.x = 0.674;
    // way_pose_1.orientation.y = -0.213;
    // way_pose_1.orientation.z = -0.213;
    // way_pose_1.orientation.w = 0.674;
    // way_pose_1.position.x = -0.020;
    // way_pose_1.position.y = -0.269;
    // way_pose_1.position.z = 0.258;

    // geometry_msgs::Pose way_pose_2;
    // way_pose_2.orientation.x = 0.693;
    // way_pose_2.orientation.y = 0.138;
    // way_pose_2.orientation.z = -0.512;
    // way_pose_2.orientation.w = 0.488;
    // way_pose_2.position.x = -0.223;
    // way_pose_2.position.y = -0.127;
    // way_pose_2.position.z = 0.388;

    // geometry_msgs::Pose way_pose_3;
    // way_pose_3.orientation.x = 0.487;
    // way_pose_3.orientation.y = -0.099;
    // way_pose_3.orientation.z = -0.618;
    // way_pose_3.orientation.w = 0.610;
    // way_pose_3.position.x = -0.213;
    // way_pose_3.position.y = 0.151;
    // way_pose_3.position.z = 0.501;

    geometry_msgs::Pose end_pose;
    end_pose.orientation.x = -0.131;
    end_pose.orientation.y = 0.695;
    end_pose.orientation.z = 0.695;
    end_pose.orientation.w = -0.131;
    end_pose.position.x = -0.287;
    end_pose.position.y = -0.049;
    end_pose.position.z = 0.347;

    /**
     * @brief joint move, cartesian target
     * 
     */

    while(true)
    {
        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        arm.setPoseTarget(start_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);
        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");

        //让机械臂按照规划的轨迹开始运动。
        if (success)
        {
            ROS_INFO("move to start pose");
            arm.execute(plan);
            ROS_INFO("reach start pose");
            break;
        }
        else
        {
            ROS_INFO("Failed to plan path to start pose, try again");
        }
    }

    /**
     * @brief cartesian move, cartesian target
     * 
     */

    // 设置路点列表
	std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);
    // waypoints.push_back(way_pose_1);
    // waypoints.push_back(way_pose_2);
    // waypoints.push_back(way_pose_3);
    waypoints.push_back(end_pose);

    ROS_INFO("start cartesian path planning demo");
    while(true)
    {
        // 笛卡尔空间下的路径规划
        moveit_msgs::RobotTrajectory trajectory;    // 规划后获得的路径
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = 0.0;  // 规划成功的路点在路点列表中的比例 [0, 1]
        int maxtries = 100;     // 最大尝试规划次数
        int attempts = 0;       // 已经尝试规划次数
        while(fraction < 1.0 && attempts < maxtries)
        {
            // 计算笛卡尔空间轨迹
            fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
            attempts++;
            
            if(attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }
        
        // 判断规划路点覆盖率
        ROS_INFO_STREAM("fraction: " << fraction);
        if(fraction == 1)
        {   
            ROS_INFO("Path computed successfully. Moving the arm.");

            // 生成机械臂的运动规划数据
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            // 执行运动
            ROS_INFO("start cartesian path plan");
            arm.execute(plan);
            ROS_INFO("finish cartesian path plan");
            sleep(1);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }
    }
}

/**
 * @brief moveit avoid obstacle demo
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // ros init
	ros::init(argc, argv, "moveit_cartesian_demo");

    // 多线程接受Topic, 防止消息堵塞
	ros::AsyncSpinner spinner(1);   
	spinner.start();

    /**
     * @brief MoveGroupInterface
     * 
     */

    // 初始化 moveit 规划组接口的实例
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    ROS_INFO_STREAM("end_effector_link: " << end_effector_link);

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalJointTolerance(0.01);
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(1.0);
    arm.setMaxVelocityScalingFactor(1.0);

    /**
     * @brief PlanningSceneInterface
     * 
     */

    // 初始化场景规划接口
    moveit::planning_interface::PlanningSceneInterface scene;
    std::map<std::string, moveit_msgs::CollisionObject> obj_vec = scene.getObjects();
    std::vector<std::string> obj_ids;
    for (std::map<std::string, moveit_msgs::CollisionObject>::iterator it = obj_vec.begin(); it != obj_vec.end(); it++)
    {
        ROS_INFO_STREAM("remove scene obj: " << it->first << std::endl);
        obj_ids.push_back(it->first);
    }
    scene.removeCollisionObjects(obj_ids);
    sleep(5.0); // 等待对象实例化

    // 设置场景碰撞对象
    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "obj_cylinder";                               // 碰撞对象ID
    shape_msgs::SolidPrimitive primitive_cylinder;              // 碰撞对象实体
    primitive_cylinder.type = primitive_cylinder.CYLINDER;        
    primitive_cylinder.dimensions.resize(3);
    primitive_cylinder.dimensions[0] = 1.0;
    primitive_cylinder.dimensions[1] = 0.2;
    primitive_cylinder.dimensions[1] = 0.2;
    geometry_msgs::Pose pose_cylinder;                          // 碰撞对象位姿
    pose_cylinder.orientation.w = 1.0;
    pose_cylinder.position.x =  0.0;
    pose_cylinder.position.y = -0.4;
    pose_cylinder.position.z =  primitive_cylinder.dimensions[0]/2.0; 
    cylinder.primitives.push_back(primitive_cylinder);
    cylinder.primitive_poses.push_back(pose_cylinder);
    cylinder.operation = cylinder.ADD;                          // 碰撞对象操作
    cylinder.header.frame_id = reference_frame;                 // 碰撞对象绑定坐标系
    std::vector<moveit_msgs::CollisionObject> collision_objects;    
    collision_objects.push_back(cylinder);
    scene.addCollisionObjects(collision_objects);               // 添加碰撞对象到场景
    sleep(5.0); // 等待对象实例化

    // 设置机械臂夹爪
    moveit_msgs::CollisionObject grasper;
    grasper.id = "grasper";
    shape_msgs::SolidPrimitive primitive_grasper;
    primitive_grasper.type = primitive_grasper.BOX;
    primitive_grasper.dimensions.resize(3);
    primitive_grasper.dimensions[0] = 0.2;
    primitive_grasper.dimensions[1] = 0.02;
    primitive_grasper.dimensions[2] = 0.02;
    geometry_msgs::Pose pose_grasper;                      
    pose_grasper.orientation.w = 1.0;
    pose_grasper.position.x =  primitive_grasper.dimensions[0] / 2.0 - 0.025;
    pose_grasper.position.y = -0.015;
    pose_grasper.position.z =  0.0; 
    grasper.primitives.push_back(primitive_grasper);
    grasper.primitive_poses.push_back(pose_grasper);
    grasper.operation = grasper.ADD;                
    grasper.header.frame_id = end_effector_link;          
    moveit_msgs::AttachedCollisionObject attached_grasper;
    attached_grasper.link_name = end_effector_link;
    attached_grasper.object = grasper;
    scene.applyAttachedCollisionObject(attached_grasper);
    sleep(5.0); // 等待对象实例化

    // joint move, joint target
    moveit_jointmove_joint_target(arm);

    // cartesian move, cartesian target
    // moveit_cartmove_pose_target(arm, end_effector_link);

    return 0;
}
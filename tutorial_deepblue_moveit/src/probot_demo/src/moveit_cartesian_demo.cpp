/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char **argv)
{
    // ros init
	ros::init(argc, argv, "moveit_cartesian_demo");

    // 多线程接受Topic, 防止消息堵塞
	ros::AsyncSpinner spinner(1);   
	spinner.start();

    // 初始化 moveit 规划组接口的实例
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂先回到初始化位置
    ROS_INFO("start home");
    arm.setNamedTarget("home");
    arm.move();
    ROS_INFO("finish home");
    sleep(1);

    // 获取当前位姿数据作为机械臂运动的起始位姿
    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;

    // 设置路点列表
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(start_pose);
    start_pose.position.z -= 0.2;
	waypoints.push_back(start_pose);
    start_pose.position.x += 0.1;
	waypoints.push_back(start_pose);
    start_pose.position.y += 0.1;
	waypoints.push_back(start_pose);

	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;    // 规划后获得的路径
	const double jump_threshold = 0.0;  // 最小移动值
	const double eef_step = 0.01;       // 终端步进值
	double fraction = 0.0;  // 轨迹规划成功比例
    int maxtries = 100;     // 最大尝试规划次数
    int attempts = 0;       // 已经尝试规划次数
    while(fraction < 1.0 && attempts < maxtries)
    {
        // 计算笛卡尔空间轨迹
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    // 判断规划路点覆盖率
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;

	    // 执行运动
        ROS_INFO("start plan");
	    arm.execute(plan);
        ROS_INFO("finish plan");
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // 控制机械臂回到初始化位置
    ROS_INFO("start home");
    arm.setNamedTarget("home");
    ROS_INFO("finish home");
    arm.move();
    sleep(1);

	ros::shutdown(); 
	return 0;
}

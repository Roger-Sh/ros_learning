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

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "moveit_fk_demo");

    // 多线程接受Topic, 防止消息堵塞
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 moveit 规划组接口的实例
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 设置规划组规划参数
    arm.setGoalJointTolerance(0.001);         // 关节允许误差
    arm.setMaxAccelerationScalingFactor(0.2); // 最大加速度比例系数
    arm.setMaxVelocityScalingFactor(0.2);     // 最大速度比例系数

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    ROS_INFO("start home");
    arm.move();
    ROS_INFO("finish home");
    sleep(1);

    // 设置关节运动目标
    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];
    arm.setJointValueTarget(joint_group_positions);

    // 执行关节运动
    ROS_INFO("start joint target 1");
    arm.move();
    ROS_INFO("finish joint target 1");
    sleep(1);

    // 控制机械臂回到初始化位置
    ROS_INFO("start home");
    arm.setNamedTarget("home");
    ROS_INFO("stop home");
    arm.move();
    sleep(1);

    ros::shutdown();

    return 0;
}

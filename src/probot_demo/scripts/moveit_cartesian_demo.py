#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 回到初始位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 获取当前位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
        waypoints = []
        waypoints.append(deepcopy(start_pose))

        # 设置路点轨迹
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.15
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.x -= 0.15
        wpose.position.y -= 0.1
        waypoints.append(deepcopy(wpose))

        # 笛卡尔路径规划
        fraction = 0.0
        maxtries = 100
        attempts = 0
        
        arm.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            # 根据实际API签名调用
            (plan, fraction) = arm.compute_cartesian_path(
                waypoints,          # 路点列表
                0.01,               # eef_step (末端执行器步长)
                True,               # avoid_collisions (避障)
                None                # path_constraints (路径约束)
            )
            
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo(f"Planning attempt {attempts}, completion: {fraction*100:.1f}%")

        if fraction == 1.0:
            rospy.loginfo("Cartesian path planning succeeded. Executing...")
            arm.execute(plan, wait=True)
            rospy.loginfo("Execution completed")
        else:
            rospy.logerr(f"Planning failed after {attempts} attempts (completion: {fraction*100:.1f}%)")

        # 回到初始位置
        arm.set_named_target('home')
        arm.go()
        moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
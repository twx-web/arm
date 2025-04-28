#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 版权所有 2019 武汉普赛微科技有限公司
#
# 根据Apache许可证2.0版授权
# 详情请见：http://www.apache.org/licenses/LICENSE-2.0

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from copy import deepcopy

def scale_trajectory_speed(traj, scale):
    """调整轨迹速度
    
    参数:
        traj: 输入轨迹(可能是RobotTrajectory或plan()返回的元组)
        scale: 速度缩放因子(0.0-1.0)
    返回:
        缩放后的新轨迹
    """
    # 处理不同类型的输入
    if isinstance(traj, tuple):
        # 如果是plan()返回的元组，提取轨迹部分
        traj = traj[1]
    
    # 创建新轨迹对象
    new_traj = RobotTrajectory()
    
    # 初始化新轨迹与输入轨迹相同
    new_traj.joint_trajectory = traj.joint_trajectory
    
    # 获取涉及的关节数量
    n_joints = len(traj.joint_trajectory.joint_names)
    
    # 获取轨迹中的点数
    n_points = len(traj.joint_trajectory.points)
    
    # 存储轨迹点
    points = list(traj.joint_trajectory.points)
    
    # 遍历所有点和关节，缩放时间、速度和加速度
    for i in range(n_points):
        point = JointTrajectoryPoint()
        
        # 关节位置不需要缩放
        point.positions = traj.joint_trajectory.points[i].positions

        # 缩放该点的时间
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        
        # 获取该点的关节速度
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        
        # 获取该点的关节加速度
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        
        # 缩放每个关节在该点的速度和加速度
        for j in range(n_joints):
            if j < len(point.velocities):
                point.velocities[j] = point.velocities[j] * scale
            if j < len(point.accelerations):
                point.accelerations[j] = point.accelerations[j] * scale * scale
    
        # 存储缩放后的轨迹点
        points[i] = point

    # 将修改后的点赋给新轨迹
    new_traj.joint_trajectory.points = points

    return new_traj

class MoveSpeedDemo:
    def __init__(self):
        # 初始化move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_speed_demo')
        
        # 初始化机械臂控制组
        arm = MoveGroupCommander('manipulator')
        
        # 获取末端执行器link名称
        end_effector_link = arm.get_end_effector_link()
        
        # 允许重新规划以提高求解成功率
        arm.allow_replanning(True)
        
        # 设置位置(米)和姿态(弧度)的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
        
        # 让机械臂回到SRDF中定义的"home"姿态
        arm.set_named_target('home')
        arm.go()
        
        # 设置机械臂目标位置(单位：弧度)
        joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        arm.set_joint_value_target(joint_positions)
        
        # 控制机械臂完成运动
        arm.go()

        # 再次回到home姿态
        arm.set_named_target('home')
        arm.go()

        # 获取规划后的轨迹
        arm.set_joint_value_target(joint_positions)
        traj = arm.plan()
        
        # 将轨迹速度缩放为原来的0.25倍
        new_traj = scale_trajectory_speed(traj, 0.25)

        # 执行缩放后的轨迹
        arm.execute(new_traj, wait=True)
        rospy.sleep(1)

        # 最后回到home姿态
        arm.set_named_target('home')
        arm.go()

        # 清理MoveIt资源
        moveit_commander.roscpp_shutdown()
        
        # 退出程序
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveSpeedDemo()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('xarm6')
        
        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # 移除场景中之前运行残留的物体
        scene.remove_world_object('eaibot_n1')
        # 设置box的高度
        eaibot_n1_height = 0
        # 设置box的三维尺寸
        eaibot_n1_size = [0.65, 0.7, 0.01]
        # 将box加入场景当中
        eaibot_n1_pose = PoseStamped()
        eaibot_n1_pose.header.frame_id = 'link_base'
        eaibot_n1_pose.pose.position.x = 0.0
        eaibot_n1_pose.pose.position.y = 0.0
        eaibot_n1_pose.pose.position.z = eaibot_n1_height + eaibot_n1_size[2] / 2.0
        eaibot_n1_pose.pose.orientation.w = 1.0
        scene.add_box('eaibot_n1', eaibot_n1_pose, eaibot_n1_size)
        rospy.sleep(2)

        # 移除场景中之前运行残留的物体
        scene.remove_world_object('ground')
        # 设置box的高度
        ground_height = -0.3
        # 设置box的三维尺寸
        ground_size = [2, 2, 0.01]
        # 将box加入场景当中
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = 'link_base'
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = ground_height + ground_size[2] / 2.0
        ground_pose.pose.orientation.w = 1.0
        scene.add_box('ground', ground_pose, ground_size)
        rospy.sleep(2)
         
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [-0.001745, 1.394518, -1.9059, -0.013963, 0.513127, 0.005236]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass

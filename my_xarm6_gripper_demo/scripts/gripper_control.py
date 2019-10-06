#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from xarm_msgs.srv import GripperConfig, GripperConfigRequest
from xarm_msgs.srv import GripperMove, GripperMoveRequest

def gripper_control(gripper_pos):
	# ROS节点初始化
    rospy.init_node('gripper_control')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/xarm/gripper_config')
    rospy.wait_for_service('/xarm/gripper_move')
    try:
        gripper_config = rospy.ServiceProxy('/xarm/gripper_config', GripperConfig)
        gripper_move = rospy.ServiceProxy('/xarm/gripper_move', GripperMove)

		# 请求服务调用，输入请求数据
        gripper_config(1500) # Set speed
        response = gripper_move(gripper_pos) # Set position
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    gripper_pos = sys.argv[1]
	#服务调用并显示调用结果
    gripper_control(gripper_pos)



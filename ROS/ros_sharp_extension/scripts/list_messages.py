#!/usr/bin/env python
import rospy
import rosmsg
import rospkg
from ros_sharp_extension.srv import *

def list_messages(req):
	#code copied from rosmsg_cmd_list in package rosmsg
	rospack = rospkg.RosPack()
	resultList = []
	packs = sorted([x for x in rosmsg.iterate_packages(rospack, '.msg')])
	for (p, direc) in packs:
		for file in rosmsg._list_types(direc, 'msg', '.msg'):
			resultList.append("%s/%s"%(p, file))
	return ListMessagesResponse(resultList)

def init():
	rospy.init_node('list_messages')
	s = rospy.Service('ros_sharp_extension/list_messages', ListMessages, list_messages)
	rospy.spin()

if __name__ == "__main__":
	init()

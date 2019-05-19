#!/usr/bin/env python
import rospy
import rosmsg
from ros_sharp_extension.srv import *

def list_messages_in_package(req):
	resultList = rosmsg.list_types(req.package_name, '.msg')
	return ListMessagesInPackageResponse(resultList)

def init():
	rospy.init_node('list_messages_in_package')
	s = rospy.Service('ros_sharp_extension/list_messages_in_package', ListMessagesInPackage, list_messages_in_package)
	rospy.spin()

if __name__ == "__main__":
	init()

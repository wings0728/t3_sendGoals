#!/usr/bin/env python
import rospy
import socket
from __init__ import *
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node("ping_node")
	command_pub = rospy.Publisher("ping_command", String, queue_size = 10)
	r = rospy.Rate(1)
	print TCP_IP, TCP_PORT, socket_timeout
	try:
		while not rospy.is_shutdown():
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.settimeout(socket_timeout)
			cmd = String()

			try:
				
				s.connect((TCP_IP, TCP_PORT))
				rospy.loginfo("Connected to Server")
				cmd.data = "resume"	

			except socket.error:
				rospy.logerr("Connection to Server Error")
				cmd.data = "pause"
				
			command_pub.publish(cmd)
			s.close()
			r.sleep()
	except rospy.ROSInterruptException:
		pass
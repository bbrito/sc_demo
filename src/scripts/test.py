#!/usr/bin/env python

import rospy
import smach
import smach_ros
from nav_msgs.srv import *
from cob_object_detection_msgs.msg import *

from nav_msgs.msg import Odometry

from simple_script_server import *
sss = simple_script_server()
from sensor_msgs.msg import Range

from message_filters import TimeSynchronizer, Subscriber
import message_filters

is_holding= False

def callback(msg1,msg2,msg3,msg4):
	
	average=(msg1.range+msg2.range+msg3.range+msg4.range)/4
	rospy.loginfo("average: %f",average)
	r = 0.1
	if (average< r) and (average >= 0.0):
		is_holding=True

	

def excute():
	rospy.init_node('test')
	rospy.loginfo("flag1...")
	
	sub1=message_filters.Subscriber("/tray_sensors/range_1", Range)
	sub2=message_filters.Subscriber("/tray_sensors/range_2", Range)
	sub3=message_filters.Subscriber("/tray_sensors/range_3", Range)
	sub4=message_filters.Subscriber("/tray_sensors/range_4", Range)
	rospy.loginfo("flag2...")
	#tss = message_filters.TimeSynchronizer([sub1,sub2],10)
	tss=message_filters.ApproximateTimeSynchronizer([sub1, sub2,sub3,sub4],10,1)
	rospy.loginfo("flag3...")
	tss.registerCallback(callback)
	rospy.loginfo("flag4...")
	
	r = 0.1
	if is_holding:
		#is_holding = True
		rospy.loginfo("holding object..")
		sub1.unregister()
		sub2.unregister()
		sub3.unregister()
		sub4.unregister()
	rospy.spin()
	
	
	
if __name__ == '__main__':
	excute()

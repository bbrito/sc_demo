#!/usr/bin/python

import rospy
import smach
import smach_ros

from nav_msgs.srv import *

from simple_script_server import *
sss = simple_script_server()
from sensor_msgs.msg import Range 

from message_filters import *
import message_filters

class Init(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['ok','failed'])

	def execute(self, userdata):
		
		sss.init("tray")
		sss.init("head")
		sss.init("torso")
		sss.init("arm")
		sss.init("base")
		sss.recover("tray")
		sss.recover("head")
		sss.recover("torso")
		sss.recover("arm")
		sss.recover("base")
		
		
		sss.move("head","front")
		sss.move("arm","home")
		sss.move("tray","deliverdown")
		sss.move("base","home")
		return "ok"






class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('STATE',Init(),
					transitions={'ok':'ended',
					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('sm_pick')
	sm = SM()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()

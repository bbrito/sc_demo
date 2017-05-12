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


from init import *
from welcome import *
from preparecoffee import *
from topeople import *



class Move(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
		with self:
			
			smach.StateMachine.add('INIT',Init(),
			  transitions={'ok':'WELCOME',
						   'failed':'failed'})

			smach.StateMachine.add('WELCOME',Welcome(),
			  transitions={'reached':'PREPARE_COFFEE',
						   'not_reached':'WELCOME',
						   'failed':'failed'})
						   
			smach.StateMachine.add('PREPARE_COFFEE',PrepareCoffee(),
			  transitions={'reached':'TO_PEOPLE',
						   'not_reached':'PREPARE_COFFEE',
						   'failed':'failed'})

			smach.StateMachine.add('TO_PEOPLE',ToPeople(),
			  transitions={'reached':'finished',
						   'not_reached':'TO_PEOPLE',
						   'failed':'failed'})






class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('STATE',Move(),
					transitions={'finished':'ended',
					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('sm_pick')
	sm = SM()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()

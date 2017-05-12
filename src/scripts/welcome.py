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




class Entrance(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])

	def execute(self, userdata):
		#s111=sss.move("base", "door")
		#s112=sss.move("base", [0, 6.4, 2.0])
		s1=sss.move("base", "entrance1")
		
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s1.get_state() == 3): 
				return 'reached'
			# finished with aborted
			elif (s1.get_state() == 4): 
				sss.stop("base")
				s112=sss.move("base", [1.05,1.38,1.4])
				if (s112.get_state() == 3): 
					s111=sss.move("base", [0, 6.4, 2.0])
					if (s111.get_state() == 3):
						sss.move("base", [-3.24,1.16,2.8])
					else:
						sss.move("base", [-0.94,5.4,1.7])
					return 'not_reached'
				else:
					return 'not_reached'
			# finished with preempted or canceled
			elif (s1.get_state() == 2) or (s1.get_state() == 8): 
				sss.stop("base")
				s112=sss.move("base", [1.05,1.38,1.4])
				if (s112.get_state() == 3): 
					s111=sss.move("base", [0, 6.4, 2.0])
					if (s111.get_state() == 3):
						sss.move("base", [-3.24,1.16,2.8])
					else:
						sss.move("base", [-0.94,5.4,1.7])
					return 'not_reached'
				else:
					return 'not_reached'
					
					
			# return with error
			elif (s1.get_error_code() > 0): 
				print "error_code = " + str(s1.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'


class Action(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])
			
	def execute(self, userdata):
		s2=sss.move("torso", "shake")
		s3=sss.move("arm", "wave", False)
		s4=sss.move("tray", "displayup")
		s3.wait()
		
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s2.get_state() == 3 ) and (s3.get_state() == 3 ) and ( s4.get_state() == 3):
				return 'reached'
			# finished with aborted
			elif (s2.get_state() == 4 )or ( s3.get_state() == 4 ) or (s4.get_state() == 4):
				sss.stop("torso")
				sss.stop("arm")
				sss.stop("tray")
				return 'not_reached'
			# finished with preempted or canceled
			elif (s2.get_state() == 2) or (s2.get_state() == 8) or (s3.get_state() == 2) or (s3.get_state() == 8) or (s4.get_state() == 2) or (s4.get_state() == 8):
				sss.stop("torso")
				sss.stop("arm")
				sss.stop("tray")
				return 'not_reached'
			# return with error
			elif (s2.get_error_code() > 0) or (s3.get_error_code() > 0) or (s4.get_error_code() > 0):
				print "error_code = " + str(s2.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'





class Welcome(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['reached','not_reached','failed'])
		with self:

			smach.StateMachine.add('ENTRANCE',Entrance(),
			  transitions={'reached':'ACTION',
						   'not_reached':'ENTRANCE',
						   'failed':'failed'})
						   
			smach.StateMachine.add('ACTION',Action(),
			  transitions={'reached':'reached',
						   'not_reached':'ACTION',
						   'failed':'failed'})



class Move(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
		with self:
			

			smach.StateMachine.add('WELCOME',Welcome(),
			  transitions={'reached':'finished',
						   'not_reached':'WELCOME',
						   'failed':'failed'})
						   



class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('STATE',Welcome(),
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

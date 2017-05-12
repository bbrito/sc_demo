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



class Kitchen(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])
	def execute(self, userdata):
		sss.move("tray","deliverdown")
		#sss.move("base", [0, 6.4, 0])
		#sss.move("base", [1.0,1.53,1.57])
		s5=sss.move("base", [-1.567,2.3,1.5])
		sss.move("torso", "shake")
		
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s5.get_state() == 3):
				return 'reached'
			# finished with aborted
			elif (s5.get_state() == 4):
				sss.stop("base")
				
				s51=sss.move("base", [0, 6.4, -1.5])
				sss.move("base", [1.4, 4.26, -1.5])
				s52=sss.move("base", [1.0,0.31,-1.57])
				if (s5.get_state() == 3):
					sss.move("base", [-1.567,2.3,1.5])
				else:
					sss.move("base", [0.23,0.67,1.8])
					return 'not_reached'
					
				return 'not_reached'
			# finished with preempted or canceled
			elif (s5.get_state() == 2) or (s5.get_state() == 8):
				sss.stop("base")
				
				s51=sss.move("base", [0, 6.4, -1.5])
				sss.move("base", [1.4, 4.26, -1.5])
				s52=sss.move("base", [1.0,0.31,-1.57])
				if (s5.get_state() == 3):
					sss.move("base", [-1.567,2.3,1.5])
				else:
					sss.move("base", [0.23,0.67,1.8])
					return 'not_reached'
					
				return 'not_reached'
			# return with error
			elif (s5.get_error_code() > 0):
				print "error_code = " + str(s5.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'
		
		
class Pregrasp(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])

	def execute(self, userdata):
		#sss.move("base", [-1.56,7.47,2.8])
		s14=sss.move("base", "kitchen")
		#s6=sss.move("arm", "pregrasp", False)
		#s12=sss.move("tray", "deliverup")
		#s6.wait()
		#s15=sss.move("arm", "grasp")
		
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s14.get_state() == 3):
				s6=sss.move("arm", "pregrasp", False)
				s12=sss.move("tray", "deliverup")
				s6.wait()
				s15=sss.move("arm", "grasp")
				return 'reached'
			# finished with aborted
			elif (s14.get_state() == 4):
				sss.stop("base")
				return 'not_reached'
			# finished with preempted or canceled
			elif (s14.get_state() == 2) or (s14.get_state() == 8):
				sss.stop("arm")
				return 'not_reached'
			# return with error
			elif (s14.get_error_code() > 0):
				print "error_code = " + str(s14.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'
		
		
		
class ToTray(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])
		

	def execute(self, userdata):
		s7=sss.move("arm", "pre-to-tray")
		s16=sss.move("arm", "to-tray")
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s7.get_state() == 3) and (s16.get_state() == 3):
				return 'reached'
			# finished with aborted
			elif (s7.get_state() == 4) or (s16.get_state() == 4):
				sss.stop("arm")
				return 'not_reached'
			# finished with preempted or canceled
			elif (s7.get_state() == 2) or (s7.get_state() == 8) or (s16.get_state() == 2) or (s16.get_state() == 8):
				sss.stop("arm")
				return 'not_reached'
			# return with error
			elif (s7.get_error_code() > 0) or (s16.get_error_code() > 0):
				print "error_code = " + str(s7.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'
		

class PrepareCoffee(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['reached','not_reached','failed'])
		with self:

			smach.StateMachine.add('KITCHEN',Kitchen(),
			  transitions={'reached':'PREGRASP',
						   'not_reached':'KITCHEN',
						   'failed':'failed'})
						   
			smach.StateMachine.add('PREGRASP',Pregrasp(),
			  transitions={'reached':'TO_TRAY',
						   'not_reached':'PREGRASP',
						   'failed':'failed'})
					
			smach.StateMachine.add('TO_TRAY',ToTray(),
			  transitions={'reached':'reached',
						   'not_reached':'TO_TRAY',
						   'failed':'failed'})

class Move(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
		with self:
			smach.StateMachine.add('PREPARE_COFFEE',PrepareCoffee(),
			  transitions={'reached':'finished',
						   'not_reached':'PREPARE_COFFEE',
						   'failed':'failed'})




class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('STATE',PrepareCoffee(),
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

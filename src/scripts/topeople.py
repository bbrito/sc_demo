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


class ToPosition(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])
		self.is_holding = False
		self.average=0.0
		self.r = 0.18
		
	def callback(self,msg1,msg2,msg3,msg4):
		self.average=(msg1.range+msg2.range+msg3.range+msg4.range)/4
		

	def execute(self,userdata):
		#rospy.loginfo("flag...")
		
		s81=sss.move("tray", "deliverup")
		
		if True:
			sub1=message_filters.Subscriber("/tray_sensors/range_1", Range)
			sub2=message_filters.Subscriber("/tray_sensors/range_2", Range)
			sub3=message_filters.Subscriber("/tray_sensors/range_3", Range)
			sub4=message_filters.Subscriber("/tray_sensors/range_4", Range)
			tss=message_filters.ApproximateTimeSynchronizer([sub1, sub2,sub3,sub4],10,0.3)
			tss.registerCallback(self.callback)
			rospy.loginfo("calling sensor...")
			while not ((self.average < self.r) and (self.average > 0.0)):
				rospy.loginfo("Waiting for object, average: %f",self.average)
				rospy.sleep(1)
			sub1.unregister()
			sub2.unregister()
			sub3.unregister()
			sub4.unregister()
			self.is_holding=True
			rospy.loginfo("object found on tray...")
			
			
			s8=sss.move("base", [0.23,0.67,1.8])

			while not rospy.is_shutdown():
			
			# finished with succeeded
				if (s8.get_state() == 3):
					return 'reached'
			# finished with aborted
				elif (s8.get_state() == 4):
					sss.stop("base")
					return 'not_reached'
			# finished with preempted or canceled
				elif (s8.get_state() == 2) or (s8.get_state() == 8):
					sss.stop("base")
					return 'not_reached'
			# return with error
				elif (s8.get_error_code() > 0):
					print "error_code = " + str(s8.get_error_code())
					sss.set_light("light", 'red')
					return 'failed'
					
		else:
			return 'not_reached'

class AThome(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])
		self.is_holding = False
		self.average=0.0
		self.r = 0.18
		self.single1=0
		self.single2=0
		self.single3=0
		self.single4=0
	def callback(self,msg1,msg2,msg3,msg4):
		self.average=(msg1.range+msg2.range+msg3.range+msg4.range)/4
		self.single1=msg1.range
		self.single2=msg2.range
		self.single3=msg3.range
		self.single4=msg4.range


	def execute(self,userdata):
		#rospy.loginfo("flag...")
		#sss.move("tray", "deliverup")
		sub1=message_filters.Subscriber("/tray_sensors/range_1", Range)
		sub2=message_filters.Subscriber("/tray_sensors/range_2", Range)
		sub3=message_filters.Subscriber("/tray_sensors/range_3", Range)
		sub4=message_filters.Subscriber("/tray_sensors/range_4", Range)
		tss=message_filters.ApproximateTimeSynchronizer([sub1, sub2,sub3,sub4],10,0.3)
		tss.registerCallback(self.callback)
		rospy.loginfo("calling sensor...")
		while not ((self.average > self.r) and (self.average < 0.5) and (self.single1 > self.r) and (self.single2 > self.r) and (self.single3 > self.r) and (self.single4 > self.r)):
			rospy.loginfo("Waiting for picking, average: %f",self.average)
			#rospy.sleep(1)
		sub1.unregister()
		sub2.unregister()
		sub3.unregister()
		sub4.unregister()
		self.is_holding=True
		rospy.loginfo("object has been taken...")
			
		sss.move("torso","nod")
		sss.move("head", "back")
		s10=sss.move("tray", "deliverdown")
		s8=sss.move("arm", "home")
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s8.get_state() == 3) and (s10.get_state() == 3):
				return 'reached'
			# finished with aborted
			elif (s10.get_state() == 4) or (s10.get_state() == 4):
				sss.stop("arm")
				sss.stop("tray")
				return 'not_reached'
			# finished with preempted or canceled
			elif (s8.get_state() == 2) or (s8.get_state() == 8) or (s10.get_state() == 2) or (s10.get_state() == 8):
				sss.stop("arm")
				sss.stop("tray")
				return 'not_reached'
			# return with error
			elif (s8.get_error_code() > 0) or (s10.get_error_code() > 0):
				print "error_code = " + str(s9.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'

		
class BaseHome(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['reached','not_reached','failed'])

	def execute(self, userdata):
		s11=sss.move("base", "home")
		sss.move("tray", "home")
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s11.get_state() == 3):
				return 'reached'
			# finished with aborted
			elif (s11.get_state() == 4):
				sss.stop("base")
				return 'not_reached'
			# finished with preempted or canceled
			elif (s11.get_state() == 2) or (s11.get_state() == 8):
				sss.stop("base")
				return 'not_reached'
			# return with error
			elif (s11.get_error_code() > 0):
				print "error_code = " + str(s11.get_error_code())
				sss.set_light("light", 'red')
				return 'failed'
		
		
		






class ToPeople(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['reached','not_reached','failed'])
		with self:

			smach.StateMachine.add('TO_POSITION',ToPosition(),
			  transitions={'reached':'AT_HOME',
						   'not_reached':'TO_POSITION',
						   'failed':'failed'})
						   
			smach.StateMachine.add('AT_HOME',AThome(),
			  transitions={'reached':'BASE_HOME',
						   'not_reached':'AT_HOME',
						   'failed':'failed'})
					
			smach.StateMachine.add('BASE_HOME',BaseHome(),
			  transitions={'reached':'reached',
						   'not_reached':'BASE_HOME',
						   'failed':'failed'})


class Move(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
		with self:
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

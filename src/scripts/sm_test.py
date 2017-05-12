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
				rospy.loginfo(self.r)
				#rospy.sleep(1)
			sub1.unregister()
			sub2.unregister()
			sub3.unregister()
			sub4.unregister()
			self.is_holding=True
			rospy.loginfo("object found on tray...")
			
			s8=sss.move("head", "front")
			#s8=sss.move("base", [0.23,0.67,1.8])

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
			rospy.loginfo(self.r)
			#rospy.sleep(1)
		sub1.unregister()
		sub2.unregister()
		sub3.unregister()
		sub4.unregister()
		self.is_holding=True
		rospy.loginfo("object has been taken...")
			
		sss.move("torso","nod")
		sss.move("head", "back")
		s10=sss.move("arm", "home")
		s8=sss.move("tray", "deliverdown")
		while not rospy.is_shutdown():
			
			# finished with succeeded
			if (s8.get_state() == 3):
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
			else:
				return 'failed'

class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('toposition',ToPosition(),
					transitions={'reached':'tohome',
					'not_reached': 'toposition',
					'failed':'ended'})
			smach.StateMachine.add('tohome',AThome(),
					transitions={'reached':'ended',
					'not_reached': 'tohome',
					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('sm_pick')
	sm = SM()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()


#def callback (msg):
#	is_holding = False
#	r = 0.17
#	if (msg.range > r) and (msg.range <= msg.max_range):
#		is_holding = True
#		rospy.loginfo(is_holding)
#	
	
#def excute ():
#	rospy.init_node('test')
#	rospy.Subscriber("/tray_sensors/range_2", Range, callback)
#	rospy.spin()
#	
	
	
#if __name__ == '__main__':
#	excute()

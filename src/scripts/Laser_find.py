#!/usr/bin/python

import rospy
import smach
import smach_ros
import math
import tf


from std_msgs.msg import String
from sensor_msgs.msg import LaserScan



class FindPosition(smach.State):
	range1 = []
	range2 = []
	pose = []
	value = 0.0
	lx =0.0
	ly = 0.0
	lz = 0.0
	oX = 0.0
	oY = 0.0
	mini = 0
	increment = 0
	bInitialized = False

	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['find','not_find','failed'],
			output_keys=['base_pose'])
			
		
		
		#Subscriber to /base_laser_front/scan
		#rospy.Subscriber("/base_laser_front/scan", LaserScan, self.callback)
		
		
	def callback(self,msg):
		self.mini = msg.angle_min
		self.increment = msg.angle_increment
		currentTime = rospy.Time.now()
		
		
		if bInitialized == False:
			self.range1 = msg.ranges
			lastTimeCalled = currentTime
			self.bInitialized = True
			
			print "FIRST_TIME:" + rospy.Time.now() + "," + len(self.range1)
			
		elif currentTime.sec - lastTimeCalled.sec > 6:
			self.range2 = msg.ranges
			print "TIME_NOW:" + rospy.Time.now() + "," + len(self.range2)
			
			for j in len(self.range2):
				print "FLAG:" + rospy.Time.now() + "," + j
				difference = self.range1[j] - self.range2[j]
				if  difference >=2 or difference <=-2:
					pos = j
					value = range2[j]
					print "LASER_INFO: angle_min:" + mini + "; angle_increment:" + increment + "; ranges_size:" + len(msg.ranges) + "."
					print "find object:ranges[" + pos + "]=" + value
					
					#calculate Object position:
					oX=lx + value * math.sin(mini + pos * increment)
					oY=ly + value * math.cos(mini + pos * increment)
					
					pose.append(oX)
					pose.append(oY)
					userdata.base_pose = pose
					print "FLAG3:[" + value + ", " + position + "]"
					print "Object_position:[" + oX + ", " + oY + "]"
					print "POSE:" + userdata.base_pose
					rospy.shutdown()
					
			lastTimeCalled = currentTime
			print "WAITING FOR NEXT CALLBACK:" + rospy.Time.now()
			
	def execute(self):
		rospy.init_node('find_position')
		listener = tf.TransformListener()
		for i in range(4):
			try:
				(trans,rot) = listener.lookupTransform('/map', '/base_laser_front_link', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
				
			lx = trans[0]
			ly = trans[1]
			lz = trans[2]
			print  "/base_laser_front_link: [" + lx + ", " + ly + ", " + lz + "]"
		
		rospy.Subscriber("/base_laser_front/scan", LaserScan, self.callback)
		
		rospy.spin()


if __name__ == '__main__':
	s=FindPosition()
	s.execute()

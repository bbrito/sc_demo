#! /usr/bin/env python
import time

import rospy
import actionlib

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

from simple_script_server.simple_script_server import simple_script_server

from simple_script_server import script

import tf
from geometry_msgs.msg import *
from moveit_msgs.srv import *

import simple_cartesian_interface as sci
import twist_controller_config as tcc

from tf.transformations import *

class MoveLin(object):
    
    def __init__(self, world, profile, rpy=(1.571, -0.0004, -0.0114,)):
        self._world = world
        self._profile = profile
        self._rpy = rpy
        
    def move(self, title, pos=[0.0, 0.0, 0.0], rpy=None, hold_duration=None):
        rospy.loginfo(title)
        if rpy is not None: 
            self._rpy = rpy
        pose = sci.gen_pose(pos, self._rpy)
        success = sci.move_lin(pose, self._world, self._profile)
        if hold_duration is not None: 
            time.sleep(hold_duration)
        return success
    
def move_arm_pick():
    world = "odom_combined"
    profile = Profile()
    profile.vel = 0.2
    profile.accl = 0.1
    profile.profile_type = Profile.SINOID
    
    ml = MoveLin(world, profile)

    ml.move('Moving to object position...', [ -1, -1.5, 1.2 ], hold_duration=0.5)    
   
    
    

def init_dyn_recfg():
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()
    
    cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_MANIPULABILITY)
    cli.set_config_param(tcc.LAMBDA_MAX, 0.1)
    cli.set_config_param(tcc.W_THRESH, 0.05)
    cli.set_config_param(tcc.SOLVER, tcc.TwistController_STACK_OF_TASKS)
    cli.set_config_param(tcc.K_H, 1.0)
    
    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA)
    cli.set_config_param(tcc.K_H_CA, -1.9)
    cli.set_config_param(tcc.DAMP_CA, 0.000001)
    cli.set_config_param(tcc.ACTIV_THRESH_CA, 0.1)
    
    cli.set_config_param(tcc.CONSTR_JLA, tcc.TwistController_JLA)
    cli.set_config_param(tcc.K_H_JLA, -1.0)
    cli.set_config_param(tcc.DAMP_JLA, 0.00001)
    cli.set_config_param(tcc.ACTIV_THRESH_JLA, 10.0)
    
    cli.set_config_param(tcc.KIN_EXT, tcc.TwistController_NO_EXTENSION)
    cli.set_config_param(tcc.KEEP_DIR, True)
    cli.set_config_param(tcc.ENF_VEL_LIM, True)
    cli.set_config_param(tcc.ENF_POS_LIM, True)
    
    cli.update()
    
    time.sleep(1.0)
    
    cli.set_config_param(tcc.K_H_CA, -2.0)
    cli.update()
    
    cli.close()




class WelcomeScript(script):

	def Initialize(self):

		# initialize components (not needed for simulation)
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		#self.sss.init("gripper")
		self.sss.init("base")

		# move to initial positions
		#handle_arm = self.sss.move("arm","home",False)
		#handle_torso = self.sss.move("torso","home",False)
		#handle_base = self.sss.move("base","home",False)
		#handle_sdh = self.sss.move("gripper","home",False)
		#self.sss.move("tray","home")
		#self.sss.move("tray","deliverup")
		#self.sss.move("tray","deliverdown")
		#self.sss.move("tray","displayup")
		#handle_arm.wait()
		#handle_torso.wait()
		#handle_sdh.wait()
		#handle_base.wait()
		


	def Run(self):
		
		rospy.loginfo("Pregrasping...")

		# welcome
		#handle0 = self.sss.move("base","entrance2")
		#handle0.wait()
		#handle_arm = self.sss.move("arm","wave",False)
		self.sss.move("tray","displayup")
		self.sss.move("tray","deliverdown")
		self.sss.move("tray","deliverup")
		self.sss.move("tray","home")
	
		#handle_arm.wait()
		#self.sss.move("torso","nod")
		#h1 = self.sss.move("arm","home")
		#h1.wait()
		
		#self.sss.move("arm","gazhome")
		#self.sss.move("arm","pre-to-tray")
		#self.sss.move("arm","to-tray")
		#self.sss.move("arm","gazhome")
		#self.sss.move("arm","home")
		#handle1 = self.sss.move("base","door")
		#handle1.wait()
		#if not self.sss.parse:
			#print "Please press ENTER to excute: "
		#self.sss.wait_for_input()
		
		# prepare coffee for guest
		#self.sss.move("base","kitchen")
		#handle02 = self.sss.move("base","door")
		#handle02.wait()
		#handle03 = self.sss.move("base",[6.9765, -6.7821, 0])
		#handle03.wait()
		#self.sss.move("arm","pregrasp")
		


if __name__ == '__main__':
	
	#SCRIPT1 = WelcomeScript()
	#SCRIPT1.Start()  
	
	#action_name = rospy.get_namespace()+'cartesian_trajectory_action'
	#client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
	#rospy.logwarn("Waiting for ActionServer: %s", action_name)
	#client.wait_for_server()
	#rospy.logwarn("...done")
    
	#init_dyn_recfg()
	#move_arm_pick()
	rospy.init_node('test')
	sss = simple_script_server()
	#sss.set_light("light", [1,2,1])
	sss.set_light("light", 'red')
	sss.set_light("light", 'green')
	#self.sss.say("sound", ["Here's your drink."])
	#myhandle0 = sss.move("arm","grasp-to-tray",False)
	#sss.move("tray","deliverdown")
	#myhandle0.wait()
	#sss.move("base",[2.330, -0.298, 0.418])	
	#sss.move("arm","folded")
	#sss.move("base","home")	
	
	
    

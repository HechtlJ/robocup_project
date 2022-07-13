#!/usr/bin/env python

import rospy
import smach
import smach_ros

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse, SetBool, SetBoolRequest, SetBoolResponse



class ExecuteGraspState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_successful', 'grasp_failed'])

    def execute(self, userdata):

        proxy = rospy.ServiceProxy("grabber/pick_up", Empty)
        proxy.call()

        proxy = rospy.ServiceProxy("/grabber/object_in_hand", SetBool)
        resp = proxy.call()

        if(resp.success == True):
            return 'grasp_successful'
        else:
            return 'grasp_failed'

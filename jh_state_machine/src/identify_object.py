#!/usr/bin/env python

import rospy
import smach
import smach_ros

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse, SetBool, SetBoolRequest, SetBoolResponse



class IdentifyObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):

        proxy = rospy.ServiceProxy("/grabber/identify_object", Empty)
        proxy.call()

        proxy = rospy.ServiceProxy("/grabber/raise_arm", Empty)
        proxy.call()

        proxy = rospy.ServiceProxy("/grabber/obs_after_grasp", Empty)
        proxy.call()

        return 'finished'

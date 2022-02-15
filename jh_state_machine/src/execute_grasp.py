#!/usr/bin/env python

import rospy
import smach
import smach_ros

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse



class ExecuteGraspState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'])

    def execute(self, userdata):

        proxy = rospy.ServiceProxy("grabber/pick_up", Empty)
        proxy.call()

        return 'finish'
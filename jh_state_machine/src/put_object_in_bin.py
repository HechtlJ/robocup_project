#!/usr/bin/env python

import rospy
import smach

from std_srvs.srv import Empty

class PutObjectInBinState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        proxy = rospy.ServiceProxy("/grabber/put_object_in_bin", Empty)
        proxy.call()

        return 'finished'

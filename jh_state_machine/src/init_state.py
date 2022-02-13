#!/usr/bin/env python

import rospy
import smach
import smach_ros

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        raw_input("Press button to start grabbing")
        return 'success'
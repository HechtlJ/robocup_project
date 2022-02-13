#!/usr/bin/env python

import rospy
import smach
import smach_ros

class TmpState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'])

    def execute(self, userdata):
        raw_input("Press button to end")
        return 'finish'
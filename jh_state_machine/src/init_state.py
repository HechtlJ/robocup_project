#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_srvs.srv import SetBool, SetBoolRequest

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):

        # Start pcl publishing
        rospy.wait_for_service("/segmentation/enable_send_pcl")
        send_pcl_proxy = rospy.ServiceProxy("/segmentation/enable_send_pcl", SetBool)
        send_pcl_proxy.call(True)

        raw_input("Press button to start grabbing")
        return 'success'
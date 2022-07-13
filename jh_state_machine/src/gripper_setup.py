#!/usr/bin/env python

import queue
import rospy
import smach
import smach_ros

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse, SetBool



class GripperSetupState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'])

    def execute(self, userdata):

        rospy.wait_for_service("/segmentation/enable_send_pcl")
        send_pcl_proxy = rospy.ServiceProxy("/segmentation/enable_send_pcl", SetBool)
        send_pcl_proxy.call(True)

        rospy.loginfo("Starting setup motion...")
        msg = JointTrajectory()

        msg.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]

        msg.points = [JointTrajectoryPoint()]

        msg.points[0].positions = [0.045, 0.045]
        msg.points[0].effort = [0, 0]

        msg.points[0].time_from_start = rospy.Duration(0.4)

        pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
        pub.publish(msg)

        proxy = rospy.ServiceProxy("/grabber/raise_arm", Empty)
        proxy.call()

        proxy = rospy.ServiceProxy("/grabber/obs_pre_grasp", Empty)
        proxy.call()

        return 'finish'
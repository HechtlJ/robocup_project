#!/usr/bin/env python

from sre_constants import SUCCESS
import rospy
import smach
import smach_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TurnHeadDownState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'])

    def execute(self, userdata):
        msg = JointTrajectory()
        msg.joint_names = []
        msg.joint_names.append("head_1_joint")
        msg.joint_names.append("head_2_joint")
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].time_from_start = rospy.Duration(0.6)
        msg.points[0].positions = [-0.0, -0.8]


        pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=0)
        rate = rospy.Rate(10)

        while True:
            connections = pub.get_num_connections()
            if(connections>0):
                pub.publish(msg)
                break
            rospy.loginfo("watiting for subscriber")
            rate.sleep()
        rospy.loginfo("Send head control command..")
        rospy.sleep(rospy.Duration(2))
        return "finish"


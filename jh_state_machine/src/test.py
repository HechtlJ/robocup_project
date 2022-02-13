#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    pub = rospy.Publisher('head_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = JointTrajectory()
        msg.joint_names = []
        msg.joint_names.append("head_1_joint")
        msg.joint_names.append("head_2_joint")
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].time_from_start = rospy.Duration(0.6)
        msg.points[0].positions = [-0.0, -0.8]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

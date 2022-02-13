#!/usr/bin/env python

import rospy
import smach
import smach_ros

from init_state import InitState
from tmp_state import TmpState
from turn_head_down import TurnHeadDownState


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', InitState(), 
                               transitions={'success':'TurnHeadDown'})
        
        smach.StateMachine.add('TurnHeadDown', TurnHeadDownState(), transitions={'finish':'Tmp'})

        smach.StateMachine.add('Tmp', TmpState(), 
                               transitions={'finish':'finish'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

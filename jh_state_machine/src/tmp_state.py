#!/usr/bin/env python

import smach

class AgainState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish', 'again'])

    def execute(self, userdata):
        string = raw_input("Press a and enter to do again or just enter to end\n")
        if 'a' in string:
            return 'again'
        else:
            return 'finish'
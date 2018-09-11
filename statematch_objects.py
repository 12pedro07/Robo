#!/usr/bin/env python

import rospy
import smach
import smach_ros
from judith_object_recognition import ObjectsOrganizer, get_close


def main():
    rospy.init_node('statematch')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.left2right = []
    sm.userdata.namesids = []
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('OBJRECO', ObjectsOrganizer.ObjectsOrganizer(), transitions={'success': 'TESTE', 'fail': 'outcome4'})
        smach.StateMachine.add('TESTE', get_close.get_close(), transitions={'success': 'outcome4', 'fail': 'outcome4'})
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

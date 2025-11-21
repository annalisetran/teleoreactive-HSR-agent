#!/usr/bin/env python

import rospy
import smach
import smach_ros

# Define planning state. 
class Planning(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['plan_generated', 'success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Planning')
        return 'plan_generated'

# Define looking state
class Looking(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['looking_success', 'looking_failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Looking')
        looking_result = input("Looking: ")
        if looking_result == 1:
            return 'looking_success'
        if looking_result == 0:
            return 'looking_failure'
        
class Success(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['replan', 'success'])

    def execute(self, userdata):
        rospy.loginfo('Execute state Success')
        return 'success'
    
class Failure(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['replan','failure'])

    def execute(self, userdata):
        rospy.loginfo('Execute state failure')
        return 'failure'

# Define action state
class Action(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['action_success', 'action_failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Actions')
        action_result = input("Action: ")
        if action_result == 1:
            return 'action_success'
        if action_result == 0:
            return 'action_failure'
        
def main():
    rospy.init_node("feedback_loop_state_machine")

    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['success', 'failure'])


    # Open the container
    with sm_top: 
        # Add state to the container
        smach.StateMachine.add('PLANNING', Planning(), 
                               transitions={'plan_generated':'CON'})
        
        smach.StateMachine.add('SUCCESS', Success(),
                               transitions={'success':'success',
                                            'replan':'PLANNING'})
        
        smach.StateMachine.add('FAILURE', Failure(),
                               transitions={'replan':'PLANNING',
                                            'failure': 'failure'})
        
        # Create sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['action_looking_success', 'in_progress', 'issue_detected'],
                                   default_outcome='in_progress',
                                   outcome_map={'action_looking_success':
                                                {'ACTION':'action_success',
                                                 'LOOKING': 'looking_success'},
                                                'issue_detected': {'ACTION': 'action_failure',
                                                 'LOOKING': 'looking_failure'}
                                                }
                                   )

        # Open the container
        with sm_con:
            # Add states to the concurrent container
            smach.Concurrence.add('ACTION', Action())
            smach.Concurrence.add('LOOKING', Looking())

        smach.StateMachine.add('CON', sm_con,
                               transitions={'in_progress': 'CON',
                                            'action_looking_success':'SUCCESS',
                                            'issue_detected': 'FAILURE'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

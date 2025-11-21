#!/usr/bin/env python

import rospy
import smach
# import smach_ros
from std_msgs.msg import Bool

# Define planning state. 
class Planning(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['plan_generated'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Planning')
        
        return 'plan_generated'
    
# Define success state
class Success(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Success')
        return 'success'
    
# Define failure state
class Failure(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Failure')
        return 'failure'

# Define looking state
class Looking(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['looking_success', 'looking_failure'])
        self._looking_result = None
        self._looking_sub = rospy.Subscriber('/looking', Bool, callback=self.looking_callback, queue_size=1)
    
    def looking_callback(self, msg: Bool):
        self._looking_result = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state Looking')
        if self._looking_result:
            return 'looking_success'
        if not self._looking_result:
            return 'looking_failure'

# Define action state
class Action(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['action_success', 'action_failure'])
        self._action_result = None
        self._action_sub = rospy.Subscriber('/action', Bool, callback=self.action_callback, queue_size=1)

    def action_callback(self, msg: Bool):
        self._action_result = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state Actions')
        if self._action_result:
            return 'action_success'
        if not self._action_result:
            return 'action_failure'
        
def main():
    rospy.init_node("feedback_loop_state_machine")

    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=[
        #'success', 'failure'
        ])

    # Open the container
    with sm_top: 
        # Add state to the container
        smach.StateMachine.add('PLANNING', Planning(), 
                               transitions={'plan_generated':'CON',
                                            #'success' : 'PLANNING',
                                            #'failure' : 'PLANNING'
                                            })
        
        smach.StateMachine.add('SUCCESS', Success(),
                               transitions={'success': 'PLANNING'})
        
        smach.StateMachine.add('FAILURE', Failure(),
                               transitions={'failure': 'PLANNING'})
        
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
                                            'issue_detected': 'FAILURE',
                                            })  

    # Execute SMACH plan
    outcome = sm_top.execute()

if __name__ == '__main__':
    main()
    

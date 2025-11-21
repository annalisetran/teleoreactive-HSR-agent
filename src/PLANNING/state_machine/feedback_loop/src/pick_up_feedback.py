#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose

from manip_srv.srv import approach_object, grasp_object, center_object_move
from unsw_vision_msgs.msg import DetectionList, ObjectDetection 

from queue import Queue

# Define planning state. 
class Planning(smach.State) :
    #TODO QUEUE LOGIC IS WRONG!!! NEED TO HAVE PAST ACTIONS AND RESULTS AS USERDATA
    def __init__(self):
        smach.State.__init__(self, outcomes=['plan_generated', 'failure'],
                             input_keys=['pick_up_point', 'object_class', 'action', 'looking'],
                             output_keys=['action', 'direction', 'looking','forward_distance'])
        
        
        self.action_q = Queue()
        self.action_q.put('approach')
        self.action_q.put('center_move')
        self.action_q.put('grasp')

        self.approach_dir_q = Queue()
        self.approach_dir_q.put('top')
        self.approach_dir_q.put('side')

    def execute(self, userdata):
        rospy.loginfo('Executing state Planning')
        if userdata.pick_up_point == None or userdata.object_class == None:
            return 'failure'

        if self.action_q.empty():
            self.action_q.put('approach')
            self.action_q.put('center_move')
            self.action_q.put('grasp')
            return 'failure'
        
        if self.approach_dir_q.empty():
            self.approach_dir_q.put('top')
            self.approach_dir_q.put('side')
            return 'failure'

        action = self.action_q.get()
        
        if action == 'approach' :
            userdata.action = 'approach'
            userdata.direction = self.approach_dir_q.get()
            userdata.looking = 'head'
            userdata.forward_distance = None
            rospy.loginfo(userdata.action)
            rospy.loginfo(userdata.looking)
            return 'plan_generated'
        elif action == 'center_move':
            userdata.action = 'center_move'
            userdata.looking = 'hand'
            return 'plan_generated'
        elif action == 'grasp':
            userdata.action = 'grasp'
            userdata.looking = 'head'
            return 'plan_generated'
        else:
            return 'failure'  



# Define looking state
class Looking(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['looking_success', 'looking_failure'],
                             input_keys=['looking', 'object_class'])
        
        self.hand_topic = "/unsw_vision/handcam/detections/objects"
        self.head_topic = "/unsw_vision/detections/objects"

        rospy.Subscriber(self.hand_topic, DetectionList, self.handcam_callback, queue_size=1)
        rospy.Subscriber(self.head_topic, DetectionList, self.headcam_callback, queue_size=1)

        self.hand_detections = []
        self.head_detections = []

    def handcam_callback(self, msg: DetectionList):
        self.hand_detections = msg.objects
        
    def headcam_callback(self, msg:DetectionList):
        self.head_detections = msg.objects

    def execute(self, userdata):
        detections = DetectionList()
        if userdata.looking == 'hand':
            detections = self.hand_detections
        if userdata.looking == 'head':
            detections = self.head_detections

        for object in detections:
            if object.object_class == userdata.object_class:
                
                return 'looking_success'
        return 'looking_failure'
        
# Define action state
class Action(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['action_success', 'action_failure'],
                             input_keys=['action', 'direction', 'pick_up_point', 'object_class', 'forward_distance'],
                             output_keys=['forward_distance'])
        
        rospy.wait_for_service('unsw/manip/approach_object', timeout=15)
        self.approach_service = rospy.ServiceProxy('unsw/manip/approach_object', approach_object)

        rospy.wait_for_service('/unsw/manip/hand_plane_move/center_object', timeout=15)
        self.center_service = rospy.ServiceProxy('/unsw/manip/hand_plane_move/center_object', center_object_move)

        rospy.wait_for_service('unsw/manip/grasp_object', timeout=15)
        self.grasp_service = rospy.ServiceProxy('unsw/manip/grasp_object', grasp_object)


    def execute(self, userdata):
        rospy.loginfo('Executing state Actions')
        action_result = None
        
        if userdata.action == 'approach':
            point_stamped = PointStamped()
            point_stamped.header.seq = 0
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.header.frame_id = 'base_link'
            point_stamped.point = userdata.pick_up_point
            approach_result = self.approach_service(userdata.direction , point_stamped)

            if approach_result.success:
                userdata.forward_distance = approach_result.forward_distance
            else:
                userdata.forward_distance = None
            
            action_result = approach_result.success
        
        if userdata.action == 'center_move':
            return 'action_failure'
            if userdata.object_class == None:
                return 'action_failure'
            center_result = self.center_service(userdata.object_class)
            action_result = center_result.success

        if userdata.action == 'grasp':
            if userdata.forward_distance == None:
                return 'action_failure'
            grasp_result = self.grasp_service(userdata.forward_distance)

            action_result = grasp_result.success

        if action_result == None:
            return 'action_failure'
        
        if action_result:
            return 'action_success'
        else:
            return 'action_failure'
        
class Success(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['replan', 'success'])

    def execute(self, userdata):
        rospy.loginfo('Execute state Success')
        return 'replan'
    
class Failure(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['replan','failure'])

    def execute(self, userdata):
        rospy.loginfo('Execute state failure')
        return 'replan'
    
class Test(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_test'], output_keys=['pick_up_point', 'object_class'])
    
    def execute(self, userdata):
        point = Point()
        point.x = 0.3
        point.z = 0.3
        userdata.pick_up_point = point
        userdata.object_class = 'banana'
        return 'setup_test'

        
def main():
    rospy.init_node("feedback_loop_state_machine")

    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['success', 'failure'])
    

    # Open the container
    with sm_top: 
        # Add state to the container
        smach.StateMachine.add('TEST', Test(),
                               transitions={'setup_test': 'PLANNING'})
        smach.StateMachine.add('PLANNING', Planning(), 
                               transitions={'plan_generated':'CON', 'failure':'PLANNING'})
        
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
                                                },
                                    input_keys=['action', 'looking', 'pick_up_point', 'direction', "object_class"]
                                   )

        # Open the container
        with sm_con:
            # Add states to the concurrent container
            smach.Concurrence.add('ACTION', Action(), remapping={'action':'action', 'looking':'looking'})
            smach.Concurrence.add('LOOKING', Looking(), remapping={'action':'action', 'looking':'looking'})

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

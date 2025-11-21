#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import actionlib
import tf2_ros

from std_msgs.msg import String, Int8
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from actionlib_msgs.msg import GoalStatus

from manip_srv.srv import view_pose

TABLE_VIEW_POSE = Pose((-8.733, -8.173, 0.000),(0,0,0.720,0.694))

class Talker:
    def __init__(self):
        tts_topic = rospy.get_param('/store_groceries_controller/tts_topic')
        self.talker = rospy.Publisher(tts_topic, String, queue_size=1)
    
    def talk(self, sentence):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(sentence)

class Start(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['task_started', 'waiting'])
        self.myTalker = myTalker
        start_topic = rospy.get_param("/store_groceries_controller/start_topic")
        self.task_started = False
        rospy.loginfo("init wrist sub callback")
        self.wrist_sub = rospy.Subscriber(start_topic, Int8, callback=self.wrist_callback, queue_size=1)

    def wrist_callback(self, msg: Int8):
        rospy.logwarn("Task started!")
        self.task_started = True

    def execute(self, userdata):
        if self.task_started:
            self.myTalker.talk("Storing Groceries task started. Waiting for the door to be opened.")
            return 'task_started'
        return 'waiting'
    
class DoorCheck(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['door_open', 'door_closed'])
        self.myTalker = myTalker
        self.door_open = False
        
        self.door_sub = rospy.Subscriber('/door_open_close', String, callback=self.doors_callback, queue_size=10)
        
    def doors_callback(self, msg: String):
        if msg.data == 'open':
            self.door_open = True

    def execute(self, userdata):
        if self.door_open:
            self.myTalker.talk("I detected the door opening.")
            return 'door_open'
        if not self.door_open:
            return 'door_closed'
    
class MoveToTable(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcome=['move_to_table_success', 'move_to_table_fail'])

        self.myTalker = myTalker
        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        self.table_view_pose = TABLE_VIEW_POSE

        rospy.wait_for_service("/unsw/manip/view_pose", timeout=15)
        self.view_pose_service = rospy.ServiceProxy("/unsw/manip/view_poose", view_pose)

        
    def execute(self, ud):
        self.myTalker.talk("I moving to the table now.")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose = self.table_view_pose

        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.cli.send_goal_and_wait(goal)

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            #TODO change to joint values for table. 
            arm_lift_joint = 0
            head_tilt_joint = 0
            self.view_pose_service(arm_lift_joint, head_tilt_joint)
            self.myTalker.talk("I have reached the table")
            return 'nav_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'nav_fail'
    
def main():
    rospy.init_node('dummy_store_groceries_smach')

    myTalker=Talker()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    sm = smach.StateMachine(outcomes = ["task_success"])

    with sm:
        smach.StateMachine.add('START', 
                               Start(myTalker), 
                               transitions={'task_started':'DOOR_CHECK', 
                                            'waiting': 'START'})
        smach.StateMachine.add('DOOR_CHECK',
                               DoorCheck(myTalker),
                               transitions={'door-open':'MOVE_TO_TABLE',
                                            'door-closed':'DOOR_CHECK'})
        smach.StateMachine.add('MOVE_TO_TABLE',
                               MoveToTable(myTalker),
                               transitions={'nav_fail':'MOVE_TO_TABLE',
                                            'nav_success':'VIEW_TABLE'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
        
    

#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf.transformations
"""
Class for making the robot look at a particular point
"""


class LookAt:


    def __init__(self):
        sub_topic = rospy.get_param('look_at_sub', 'look_at')
        self.sub = rospy.Subscriber(sub_topic, PoseStamped, self.lookAtCallback, queue_size=20)
        self.move_head_pub = rospy.Publisher('hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.transformer = tf.TransformListener()
        self.timer_interval = 100 # in nanoseconds
        self.move_rate = 0.1
        self.move_duration = 100 # in nanoseconds
        self.accuracy = 0.1
        self.timer = rospy.timer(rospy.Duration(nsecs=self.timer_interval), self.adjustHeadPos)
        self.last_frame_stamp = rospy.Time.now()
        self.targetPose: PoseStamped = None

        

    def transform_pose(self, pose, target_frame):
        self.transformer.waitForTransform(
            pose.header.frame_id, target_frame, pose.header.stamp, rospy.Duration(5))
        return self.transformer.transformPose(target_frame, pose)
    
    def move_head(self, x, y):
            rospy.loginfo("move head to left(robot perspective)")
            headPos = JointTrajectory()
            headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
            point1 = JointTrajectoryPoint()
            point1.positions = [x, y]
            point1.time_from_start = rospy.Duration(secs=1)
            headPos.points = [point1]
            self.move_head_pub.publish(headPos)
    
    def lookAtCallback(self, msg: PoseStamped):
        self.targetPose = msg
    
    """
    MOVE HEAD
    Topic details:
    - position: length two array
        - index 0: horizontal joint (max 1.75 left, min -3.84)
        - index 1: vertical joint (min -1.57, max 0.52) - positive = up
    """
    def adjustHeadPos(self):
        if self.targetPose == None: 
            return
        
        point: PoseStamped = self.transform_pose(self.targetPose, "head_rgbd_sensor_joint")
        move_x = 0
        move_y = 0
        if point.pose.position.x > self.accuracy:
            move_x = -self.move_rate
        elif point.pose.position.x < -self.accuracy:
            move_x = self.move_rate
        if point.pose.position.y > self.accuracy:
            move_y = -self.move_rate
        elif point.pose.position.y < -self.accuracy:
            move_y = self.move_rate
        if move_x != 0 or move_y != 0:
            self.move_head(move_x, move_y)



def main(args):
    obj = LookAt()
    rospy.init_node('LookAt', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
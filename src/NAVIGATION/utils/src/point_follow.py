#!/usr/bin/env python
import rospy
import sys
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from tf import transformations
class simple_class:

  def __init__(self):
    self.sub = rospy.Subscriber("/clicked_point", PointStamped, self.callback)
    self.posesub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.goal_callback)
    
    self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    self.reached_pub = rospy.Publisher("/move_near_reached", Bool, queue_size=1)
    self.goal = PointStamped()
    self.moving = False

  def callback(self, data: PointStamped):
    msg = GoalID()
    msg.stamp = rospy.Time.now()
    msg.id = "-1"
    self.cancel_pub.publish(msg)
    
    self.goal = data
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    
    goal.pose.position = data.point
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1
    # goal.post.orientation = tf.transformations.euler_from_quaternion()
    self.moving = True
    self.pub.publish(goal)
    
    # to be removed -- 15 seconds delay to determine it has reach goal
    rospy.sleep(15)
    if self.moving == False: return
    self.cancel_pub.publish(msg)

    self.reached_pub.publish(Bool(True))
    self.moving = False
  

  def goal_callback(self, current_pose: PoseWithCovarianceStamped):
    if self.moving == False: return;
    cur_x = current_pose.pose.pose.position.x
    cur_y = current_pose.pose.pose.position.y
    goal_x = self.goal.point.x
    goal_y = self.goal.point.y
    dx2 = math.pow(goal_x - cur_x, 2)
    dy2 = math.pow(goal_y - cur_y, 2)

    if (dx2 + dy2) < 0.35 ** 2:
      rospy.loginfo('Cancel goal, within 0.12 meter')
      msg = GoalID()
      msg.stamp = rospy.Time.now()
      msg.id = "-1"
      self.cancel_pub.publish(msg)
      # dw = math.atan(math.sqrt(dx2) / math.sqrt(dy2))
      
      # goal = PoseStamped()
      # goal.header.stamp = rospy.Time.now()
      # goal.header.frame_id = "map"
      # goal.pose = current_pose.pose.pose
      # print(f'{math.sqrt(dx2)} {math.sqrt(dy2)}')
      # print(f'{goal.pose.orientation.w} {dw} {goal.pose.orientation.w + dw}')
      # goal.pose.orientation.w += dw
      # self.pub.publish(goal)
      # rospy.sleep(5)
      self.reached_pub.publish(Bool(True))
      self.moving = False

def main(args):
  obc = simple_class()
  rospy.init_node('point_follow', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
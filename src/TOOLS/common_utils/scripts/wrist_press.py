#! /usr/bin/env python3

import rospy
import hsrb_interface
from std_msgs.msg import Int8
from geometry_msgs.msg import WrenchStamped
import sys
import signal

class GracefulKiller:
  def __init__(self, robot):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)
    self.pub = rospy.Publisher('/start_task', Int8, queue_size = 2)
    self.robot = robot
    self.wrist_sub = rospy.Subscriber("hsrb/wrist_wrench/raw", WrenchStamped, self.wristCB)

  def wristCB(self, data):
      if(data.wrench.force.x > 20):
          rospy.loginfo("wrist pressed")
          self.pub.publish(1)

  def exit_gracefully(self, signum, frame):
    self.robot.close()
    sys.exit()


if __name__ == "__main__":
    rospy.init_node('wrist_press')
    ROBOT = hsrb_interface.Robot()
    rospy.loginfo("starting wrist press")
    killer = GracefulKiller(ROBOT)
    while not rospy.is_shutdown():
        rospy.spin()



    #while True:
    #    wrench = ROBOT.get('wrist_wrench',hsrb_interface.ItemTypes.FORCE_TORQUE).wrench
    #    if wrench[0][0] > 20.0:
    #        rospy.sleep(0.02)
    #        pub.publish(1)

    #        print "wrist press"
    #    rospy.sleep(0.01)


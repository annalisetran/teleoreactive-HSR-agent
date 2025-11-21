#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8


class DoorChecker:
    def __init__(self) -> None:
        self.laser_scan_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, callback=self.laserScanCallBack, queue_size=10)
        self.open_pub = rospy.Publisher('/door_open_close', String, queue_size=10)
        self.wrist_sub = rospy.Subscriber('/start_task', Int8, callback=self.wrist_callback, queue_size=10)
        self.start = False
        self.initial_scan = None
        self.threshold = 0.2

    def wrist_callback(self, msg:Int8):
        self.start = True

    # called when recieving laser scan message
    def laserScanCallBack(self, msg: LaserScan):
        current_scan = msg.ranges[len(msg.ranges) // 2]
        if (self.start and self.initial_scan == None):
            self.initial_scan = current_scan
        if (self.initial_scan != None and current_scan - self.initial_scan > self.threshold):
            self.doorOpen()
    
    def doorOpen(self):
        rospy.sleep(4)
        self.open_pub.publish("open")
    
if __name__ == "__main__":
    rospy.init_node('DoorChecker')
    doorCheckerNode = DoorChecker()
    while not rospy.is_shutdown(): 
        rospy.sleep(1)

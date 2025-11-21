#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
import numpy as np
import sys
class LaserMerger:
    def __init__(self):
        laser1_topic = rospy.get_param("/laser_merger/laser1_topic")
        laser2_topic = rospy.get_param("/laser_merger/laser2_topic")
        laser1_sub = message_filters.Subscriber(laser1_topic, LaserScan)
        laser2_sub = message_filters.Subscriber(laser2_topic, LaserScan)
        ts = message_filters.ApproximateTimeSynchronizer([laser1_sub,laser2_sub],10,0.2)
        self.final_pub = rospy.Publisher("/merged_laser", LaserScan, queue_size=1)
        ts.registerCallback(self.mergecallback)
        #print("laser merger init done")
    def mergecallback(self, laser1: LaserScan, laser2: LaserScan):
        # ERROR CHECKING FOR TIME DELAY
        # t = (laser1.header.stamp.secs - laser2.header.stamp.secs)
        # print(t)
        
        '''
        merge laser 2 to laser 1. Basically take the shorter distance for each angle.
        '''
        range1 = np.array(laser1.ranges)
        range2 = np.array(laser2.ranges)
        # rospy.loginfo(np.max(range1))
        # rospy.loginfo(np.max(range2))
        range2 = np.append(range2, [np.inf])
        # append 1 inf so the shape matches. revise thie code later.
        merged_range = np.minimum(range1,range2)
        
        # TODO: FILTER OUT UNUSED INTENSITY. So each of them are meaningful
        # a mask indicates for which indices the intensity is still valid,
        # AKA the range1 value for that position is smaller
        # range2(at the moment from pcl) does not have intentisy so make it 0
        intensity1_mask = (merged_range == range1)
        intensity2_mask = ~intensity1_mask
        laser1.ranges = tuple(merged_range)
        self.final_pub.publish(laser1)

def main(args):
  rospy.init_node('laser_merger', anonymous=True)
  obc = LaserMerger()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
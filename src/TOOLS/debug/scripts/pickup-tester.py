#!/usr/bin/env python3
import rospy
import sys
from unsw_vision_msgs.msg import DetectionList
from unsw_action_msg.msg import UNSWActionMsg, UNSWActionResult
import json
class PickUpDemo:

  def __init__(self):
    self.attempted = False
    self.vision_sub = rospy.Subscriber('/unsw_vision/detections/objects/positions', DetectionList, self.vision_callback)
    self.action_pub = rospy.Publisher('/actions', UNSWActionMsg, queue_size=1)
    self.pickup_pub = rospy.Timer(rospy.Duration(0.5), self.on_timer)
    self.desire_class = "Drink Can"
    #self.desire_class = "Utensil"
    self.detections = []
  def vision_callback(self, data: DetectionList):
    if not self.attempted:
      self.detections = data.objects
      for detection in data.objects:
        print(f"Detected object: {detection.object_class} with confidence {detection.object_class_conf} with tracking id {detection.tracking_id}")

  def on_timer(self, event):
    if self.attempted:
      return
    
    for items in self.detections:
      if items.object_class == self.desire_class:
        tracking_id = str(items.tracking_id)
        direction = "front"
        object_class = items.object_class
        action_data = {
            'target_point' : {
                'header' : {
                    'frame_id' : 'map'
                },
                'point' : {
                    'x' : items.position.x,
                    'y' : items.position.y,
                    'z' : items.position.z
                }
            },
            'direction' : "front",
            'class_name' : object_class,
            'object_id' : tracking_id
        }
        
        # approach
        action_msg = UNSWActionMsg()
        action_msg.action_name = "manip_approach"
        action_msg.data = json.dumps(action_data)
        
        self.action_pub.publish(action_msg)
        # wait for 10 seconds
        rospy.loginfo(f"10 seconds to hit e stop")
        rospy.sleep(5)
        break


def main(args):
  rospy.init_node('pick_up_demo', anonymous=True)
  obc = PickUpDemo()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
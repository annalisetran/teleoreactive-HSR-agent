#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from unsw_vision_msgs.msg import DetectionList
from unsw_vision_msgs.msg import ObjectDetection

import copy


class ObjectsVisualiser:

    def __init__(self):
        rospy.init_node('debugger_objects_visualiser')
        self.visionSub = rospy.Subscriber('/unsw_vision/detections/objects/positions', DetectionList, self.OnDetection)
        self.debuggerPub = rospy.Publisher('/debugger/objects', MarkerArray, queue_size=10)
        self.message = None

        rospy.Timer(rospy.Duration(secs=0.3), self.DrawObjects)

        # Prep the marker
        self.marker = MarkerArray()
        print('Object Visualiser Running.')

    
    def WipeMarker(self):
        m = Marker()
        m.action = Marker.DELETEALL

        self.marker.markers.clear()
        self.marker.markers.append(m)
        self.debuggerPub.publish(self.marker)

        self.marker.markers.clear()

        


    def OnDetection(self, data:DetectionList):
        self.message = data


    def DrawObjects(self, event=None):
        if self.message is None:
            print('[Debug] Visualisation_Objects: Message not received yet.')
            return
        
        self.WipeMarker()

        object:ObjectDetection
        i = 0
        for object in self.message.objects:
            i = i + 1

            marker = Marker()
            marker.header.frame_id = self.message.header.frame_id
            marker.header.stamp = self.message.header.stamp

            # marker ID
            marker.ns = 'object'
            marker.id = i

            # marker type
            marker.action = Marker.ADD
            marker.type = Marker.CUBE

            # transform
            marker.pose.position = object.position

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0


            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.a = 1.0
            marker.color.r = 1.0

            self.marker.markers.append(marker)

            ### TEXT MARKER

            textMarker = Marker()
            textMarker.header = marker.header
            textMarker.ns = 'name'
            textMarker.id = i

            textMarker.action = Marker.ADD
            textMarker.type = Marker.TEXT_VIEW_FACING
            textMarker.pose.position = copy.copy(object.position)   # Is this necessary?
            textMarker.pose.position.z = textMarker.pose.position.z + 0.05

            textMarker.scale.x = 0.1
            textMarker.scale.y = 0.1
            textMarker.scale.z = 0.1

            textMarker.color.a = 1.0
            textMarker.color.r = 1.0
            textMarker.color.g = 1.0
            textMarker.color.b = 1.0

            textMarker.text = f'{object.object_class} [{"{:.2f}".format(object.position.x)}, {"{:.2f}".format(object.position.y)}, {"{:.2f}".format(object.position.z)}]'

            self.marker.markers.append(textMarker)
        
        self.debuggerPub.publish(self.marker)


            

        
        




        
        





if __name__ == '__main__':

    visualiser = ObjectsVisualiser()
    rospy.spin()






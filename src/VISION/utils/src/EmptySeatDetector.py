import rospy
import sys
from unsw_vision_msgs.msg import DetectionList, ObjectDetection
from diagnostic_msgs.msg import KeyValue

class EmptySeatDetector:

    sofa_capacity = 2
    def __init__(self):
        detectionIn_topic = rospy.get_param("/EmptySeatDetector/in_topic")
        republish_topic = rospy.get_param("/EmptySeatDetector/out_topic")
        if not detectionIn_topic:
            rospy.loginfo("No detection in topic for BagScanner")

        rospy.loginfo("initialise empty seat detector")
        self.sub = rospy.Subscriber(detectionIn_topic, DetectionList, self.detectionListCallback, queue_size=10)
        self.pub = rospy.Publisher(republish_topic, DetectionList,queue_size=10)
        
    def checkOverlap(self, bbox_person, bbox_seat):        
        # buttom right corner for person and seat
        person_br_x = bbox_person.x + bbox_person.width
        person_br_y = bbox_person.y + bbox_person.height
        seat_br_x = bbox_seat.x + bbox_seat.width
        seat_br_y = bbox_seat.y + bbox_seat.height
        
        # overlap in x and y
        overlap_x = min(person_br_x, seat_br_x) - max(bbox_person.x, bbox_seat.x)
        overlap_y = min(person_br_y, seat_br_y) - max(bbox_person.y, bbox_seat.y)
        
        # overlap area
        sI = max(0,overlap_x) * max(0,overlap_y)

        
        # Intersection over seat
        ios = sI / (bbox_person.width*bbox_person.height)
        # Intersection over person
        iop = sI / (bbox_seat.width*bbox_seat.height)
        return max(ios, iop)
        
    
    def detectionListCallback(self, detectionList: DetectionList):
        people = detectionList.people
        for obj in detectionList.objects:
            try:
                if obj.object_class == 'chair':
                    tag = KeyValue()
                    tag.key = ObjectDetection.KEY_MOVABLE
                    tag.value = 'true'
                    for p in people:
                        if self.checkOverlap(p.bbox_person, obj.bbox) > 0.5:
                            tag.value = 'false'
                            break
                    obj.tags.append(tag)
                elif obj.object_class == 'sofa':
                    tag = KeyValue()
                    tag.key = ObjectDetection.KEY_MOVABLE
                    tag.value = 'true'
                    
                    people_on_sofa = 0
                    for p in people:
                        if self.checkOverlap(p.bbox_person, obj.bbox) > 0.2:
                            people_on_sofa += 1
                    
                    if people_on_sofa >= self.sofa_capacity:
                        tag.value = 'false'
                    obj.tags.append(tag)
            except Exception as e:
                rospy.logwarn(f"Error in Empty Seat detectionListCallback, Still republish the msg {e}")
                self.pub.publish(detectionList)
                return
        self.pub.publish(detectionList)
                
def main(args):
    obc = EmptySeatDetector()
    rospy.init_node('EmptySeatDetector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
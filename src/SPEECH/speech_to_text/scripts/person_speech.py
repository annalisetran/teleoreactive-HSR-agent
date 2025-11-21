#!/usr/bin/env python3

# this node synchronises the multi-modal messages for the following message types:
# person recognition and speech recognition messages
# it listens to the speech recognition messages and waits for the person recognition messages
# it then publishes the person id with the speech recognition messages
# the person id is the person of interest
# the speech message will be the final output from the vosk speech recognition engine


import rospy
from unsw_vision_msgs.msg import DetectionList, PersonDetection
from speech_to_text.msg import speech_recognition, person_speech
from std_msgs.msg import String
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer, Subscriber

class PersonSpeech():
    def __init__(self):
        # list of message type subscribers to be synchronised
        # - person id - the person of interest (conditionally required)
        # - speech text - the speech text (required)
        self.person_detection_sub = Subscriber("/unsw_vision/detections/persons", DetectionList, queue_size=10)
        self.speech_recognition_sub = Subscriber("/speech_recognition/vosk_result", speech_recognition)
        self.sync_pub = rospy.Publisher("/unsw_vision/detections/persons/speech", person_speech, queue_size=5)

        ats = ApproximateTimeSynchronizer([self.speech_recognition_sub, self.person_detection_sub], queue_size=20, slop=0.2)
        ats.registerCallback(self.synchronize_callback)
        rospy.loginfo("Person Speech node initialised")


    def synchronize_callback(self, speech_msg, person_msg):
        if speech_msg.isSpeech_recognized:
            new_msg = person_speech()
            new_msg.header = Header()  
            new_msg.header.stamp  = rospy.Time.now()
            new_msg.person_id = person_msg.people[0].id
            new_msg.speech = speech_msg.final_result     
            print(f"ID: {person_msg.people[0].id}, Speech: {speech_msg.final_result}")
            self.sync_pub.publish(new_msg)              

    # def person_detection_callback(self, msg): 
    #     if len(msg.people) > 0:  
    #         self.person_id = msg.people[0].id
    #         print(f"Person Id: {self.person_id}")

    # def speech_recognition_callback(self, msg):
    #     self.speech_message = msg.final_result
    #     print(f"Speech: {self.speech_message}")


if __name__ == '__main__':
    try:
        rospy.init_node('person_speech')
        ps = PersonSpeech()
        rospy.spin()   

    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the Person Speech node...")
        rospy.sleep(1)
        print("node terminated")

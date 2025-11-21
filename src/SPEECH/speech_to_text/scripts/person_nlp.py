#!/usr/bin/env python3

# this node synchronises the multi-modal messages for the following message types:
# person recognition and speech recognition messages
# it listens to the speech recognition messages and waits for the person recognition messages
# it then publishes the person id with the speech recognition messages
# the person id is the person of interest
# the speech message will be the final output from the vosk speech recognition engine
# spacy v=3.75
# pydandtic v=1.7.4
# markupsafe v=2.01 


import rospy
import spacy
import json
#from unsw_vision_msgs.msg import DetectionList, PersonDetection
from speech_to_text.msg import speech_recognition, person_speech
from std_msgs.msg import String
# from std_msgs.msg import Header

class PersonNLP():
    def __init__(self):
        # Initialize ROS node
        rospy.loginfo("Initializing PersonNLP node")

        # Subscribers and Publishers
        self.speech_recognition_sub = rospy.Subscriber("/speech_recognition/final_result", String, self.nlp_callback, queue_size=50)
        self.nlp_pub = rospy.Publisher("/unsw_vision/detections/persons/speech", person_speech, queue_size=5)

        # Load spaCy language model
        self.nlp = spacy.load("en_core_web_sm")
        rospy.loginfo("Model 'en_core_web_sm' loaded")
        rospy.loginfo("PersonNLP node initialized")

    def nlp_callback(self, speech_msg):
        rospy.loginfo(f"Speech result: {speech_msg.data}")

        # Process the speech data with spaCy
        doc = self.nlp(speech_msg.data)
        
        # Extract nouns and verbs
        nouns, verbs = self.extract_nouns_verbs(speech_msg.data)

        # Print the results
        print("Nouns:", nouns)
        print("Verbs:", verbs)

        # Tokenize the result
        tokenized_result = ' '.join([token.text for token in doc])
        rospy.loginfo("Tokenized result: %s", tokenized_result)

        # Create and publish the new message
        new_msg = person_speech()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.person_id = 1
        new_msg.speech = tokenized_result
        self.nlp_pub.publish(new_msg)

    # Define a function to extract nouns and verbs
    def extract_nouns_verbs(self, text):
        doc = self.nlp(text)
        nouns = [token.text for token in doc if token.pos_ == "NOUN"]
        verbs = [token.text for token in doc if token.pos_ == "VERB"]
        return nouns, verbs

    def close(self):
        rospy.loginfo("Shutting down PersonNLP node")

if __name__ == '__main__':
    try:
        rospy.init_node('nlp_person')
        person_nlp = PersonNLP()
        rospy.spin()   

    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the NLP Person Speech node...")
        rospy.sleep(1)
        print("node terminated")

    finally:
        person_nlp.close()

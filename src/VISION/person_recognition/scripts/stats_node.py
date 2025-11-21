#!/usr/bin/env python3

# This node is used to collect stats about what is happening in the program.

import rospy
import os
from std_msgs.msg import String
import signal
import sys

# Class to collect statistics
# This class will subscribe to the stats topic and collect statistics about the program
# The messages will be in the format: "msg_type, match_type, person_id, tracking_id "
# msg_type: match, summary, unit_record, distance
class StatsListener():
    def __init__(self) -> None:
        # parameters        
        self.input_topic_match = "/unsw/person_recognition/stats_match"
        self.input_topic_distance = "/unsw/person_recognition/stats_distance"
        self.match_cnt_file = os.path.join(os.environ.get('UNSW_WS', ''), "VISION/person_recognition/logs/stats_match_cnt.txt") # file to write the statistics to
        self.summary_stats_file = os.path.join(os.environ.get('UNSW_WS', ''), "VISION/person_recognition/logs/stats_summary.txt") # file to write the summary statistics to
        self.unit_record_file = os.path.join(os.environ.get('UNSW_WS', ''), "VISION/person_recognition/logs/stats_unit_record.txt") # file to write the unit record to

        self.img_cnt = 0

        # statistics to collect
        self.track_match_list = []  # list of people that have been matched using yolo tracking id
        self.face_match_list = []   # list of people that have been matched using face features
        self.head_match_list = []   # list of people that have been matched using head features

        # Dictionaries to maintain counts for each match type
        self.track_count = {}
        self.face_count = {}
        self.head_count = {}

        self.stats_listener = rospy.Subscriber(self.input_topic_match, String, self.process_match, queue_size=10) # subscribe to the stats topic
        # self.stats_listener = rospy.Subscriber(self.input_topic_distance, String, self.process_distance, queue_size=10) # subscribe to the stats topic

        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo("Statistics Listener started")   

    def process_match(self, msg):
        # parse the message
        # Split the string by commas and strip whitespace
        parts = [part.strip() for part in msg.data.split(',')]
        # print(parts)
        if parts[0] == "match":
            self.add_to_list(parts[1], parts[2])
        # elif parts[0] == "summary":
        #     self.print_summary_stats(msg)


    def add_to_list(self, match_type, person_id):
        if match_type == "tracking":
            self.match_tracking(person_id)
        elif match_type == "face":
            self.match_face(person_id)
        elif match_type == "head":
            self.match_head(person_id)
        else:
            rospy.logwarn(f"Invalid match type: {match_type}")

    def match_tracking(self, person):
        # print(f"Person: {person}")
        self.track_match_list.append(person)
        if person in self.track_count:
            self.track_count[person] += 1
        else:
            self.track_count[person] = 1

    def match_face(self, person):
        self.face_match_list.append(person)
        if person in self.face_count:
            self.face_count[person] += 1
        else:
            self.face_count[person] = 1

    def match_head(self, person):
        self.head_match_list.append(person)
        if person in self.head_count:
            self.head_count[person] += 1
        else:
            self.head_count[person] = 1   

    def print_match_stats(self):    
        print("Match Statistics:")
        print("Tracking Matches:")
        for match in self.track_count:
            print(f"track_match, {match}, {self.track_count[match]}")

        print("Face Matches:")
        for match in self.face_count:
            print(f"face_match, {match}, {self.face_count[match]}")

        print("Head Matches:")
        for match in self.head_count:
            print(f"head_match, {match}, {self.head_count[match]}")


    # def print_match_stats(self):
    #     print("Match Statistics:")
    #     for match in self.track_match_list:
    #         print(f"track_match, {match}")
    #         # print(f"track_match, {match[0]}, {match[1]}")      

    #     for match in self.face_match_list:
    #         print(f"face_match, {match}")
    #         # print(f"face_match, {match[0]}, {match[1]}")

    #     for match in self.head_match_list:
    #         print(f"head_match, {match}")
    #         # print(f"head_match, {match[0]}, {match[1]}")



    def process_distance(self, msg):
        # parse the message
        # Split the string by commas and strip whitespace
        parts = [part.strip() for part in msg.data.split(',')]
        print(parts[0])
        if parts[0] == "distance":
            self.add_to_distance_list(parts[1], parts[2], parts[3])
        # elif parts[0] == "summary":
        #     self.print_summary_stats(msg)

    def print_distance_stats(self):
        print("Distance Statistics:")

        for match in self.face_match_list:
            print(f"face, {match[0]}, {match[1]} \n")

        for match in self.head_match_list:
            print(f"head, {match[0]}, {match[1]} \n")

    def print_summary_stats(self, msg):
        print("Summary Statistics:")
        for item in msg:
            print(item)


    def write_match_stats(self):
        # Writing to a file
        with open(self.match_cnt_file, 'w') as file:
            file.write(f"match_type, person_id, track_id, cnt\n")
            for match in self.track_match_list:
                file.write(f"track_match, {match[0]}, {match[1]} \n")

        
    def signal_handler(self, sig, frame):
        # Print statistics before quitting
        # self.write_statistics()
        self.print_match_stats()
        sys.exit(0)    


if __name__ == "__main__":
    try:
        rospy.init_node('stats_listener_node')
        stats = StatsListener()        
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logfatal("Error occurred! Stopping the statistics listener node...")
            rospy.sleep(1)
            print("stats_listener_node terminated")
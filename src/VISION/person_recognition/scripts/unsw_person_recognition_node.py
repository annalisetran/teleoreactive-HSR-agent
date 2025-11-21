#!/usr/bin/env python3

# Person Recoginition 
# Detects people using texture and faces and captures the name and stores in the world model database
# Libraries:
# face recognition - https://github.com/ageitgey/face_recognition

# Dependencies:
# World Model - database with schema 2.1.4 or higher which contains the agent table structures - use with flag: use_world_model = True

# Written By: Adam Golding
import rospy
import sys
from std_msgs.msg import String
import signal
import os
from sensor_msgs.msg import Image
from unsw_vision_msgs.msg import DetectionList
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from visualizer import visualize_image
# from unsw_person_recognition import PersonRecognition
from unsw_person_recognition_torch import PersonRecognition

class PersonRecogniserNode():
    def __init__(self) -> None:
        # parameters
        self.input_topic = rospy.get_param('~input_topic')                  # topic to get the image from
        self.output_topic = rospy.get_param('~output_topic')                # topic to publish the face detection info to            
        self.vis_output_topic = rospy.get_param('~vis_output_topic')        # topic to publish the visualization of the detections         
        self.save_img = rospy.get_param('~save_img', False)                 # flag indicates whether to save the image
        self.img_path = rospy.get_param('~img_path')                        # path where face image is saved
        self.display_img = rospy.get_param('~display_img',False)            # flag to indicate whether to display the image with bounding box and person name
        self.use_world_model = rospy.get_param('~use_world_model',False)    # flag to indicate whether to use the world model database
        self.db_ini = rospy.get_param('~db_ini')                            # file containing database connection details
        self.use_gpu = rospy.get_param('~use_gpu', False)                   # flag indicating whether to use a gpu for processing the image
        self.visualize = rospy.get_param('~visualize', False)               # flag indicating whether to publish a visualization of the detections
        self.queue_size = rospy.get_param('~queue_size')                    # the queue size for the publishers
        self.scale = float(rospy.get_param('~scale'))                       # the amount to scale the image, must be a float, e.g. 0.25, 1.0
        self.face_threshold = float(rospy.get_param('~face_threshold'))   # the threshold to determine if comparing 2 faces are the same
        self.feature_threshold = float(rospy.get_param('~feature_threshold'))   # the threshold to determine if comparing 2 histograms are the same
        self.frame_count = 0       # used to determine which frame is being processed 
        self.use_statistics = True      # flag to indicate whether to use the statistics node   
        self.stats_topic = "/unsw/person_recognition/stats" # topic to publish the statistics to             
                                                  
        # log info
        rospy.loginfo("Parameters Set")
        rospy.loginfo("input_topic="+self.input_topic)
        rospy.loginfo("output_topic="+self.output_topic)
        rospy.loginfo("vis_output_topic="+self.vis_output_topic)                
        rospy.loginfo("display_img="+str(self.display_img))
        rospy.loginfo("img_path="+str(self.img_path))
        rospy.loginfo("display_img="+str(self.display_img))
        rospy.loginfo("use_world_model="+str(self.use_world_model))
        rospy.loginfo("use_gpu="+str(self.use_gpu))
        rospy.loginfo("db_ini="+self.db_ini)
        rospy.loginfo("visualize="+str(self.visualize))
        rospy.loginfo("queue_size="+str(self.queue_size))       
        rospy.loginfo("scale="+str(self.scale))
        rospy.loginfo("face_threshold="+str(self.face_threshold))
        rospy.loginfo("feature_threshold="+str(self.feature_threshold))

        rospy.loginfo("\nInitialising Person Recognition Node")

        self.pr = PersonRecognition(self.db_ini, self.scale, self.feature_threshold, self.face_threshold, self.use_world_model, self.use_gpu,
                                     self.save_img, self.img_path)
        

        # subscribers
        self.image_listener = rospy.Subscriber(self.input_topic, DetectionList, self.recognise, queue_size=600) # subscribe to the img topic

        # publishers
        self.person_publisher = rospy.Publisher(self.output_topic, DetectionList, queue_size=self.queue_size)

        signal.signal(signal.SIGINT, self.signal_handler)
        
        if self.visualize:
            # The publisher for the visualization of the detections
            self.vis_publisher = rospy.Publisher(self.vis_output_topic, Image, queue_size=self.queue_size) 

        if self.use_statistics:
            self.stats_publisher = rospy.Publisher(self.stats_topic, String, queue_size=self.queue_size)

        rospy.loginfo("Person Recogniser Node started\n")   

    '''   
    Callback Method 
    Method processes each image frame to determine, record and publish person information in the vision message
    '''
    def recognise(self, in_msg): 
        self.frame_count += 1
        print(f"\n**New Image: {self.frame_count}; Header = {in_msg.header}**")        
        
        # if we have people detected in the image then we will continue to process the image        
        if len(in_msg.people) > 0:    
            new_msg = self.pr.process_scene(in_msg)
            # publish the new message to output topic

            self.person_publisher.publish(new_msg)

            if self.visualize:
                out_msg = visualize_image(new_msg.image, new_msg)
                self.vis_publisher.publish(out_msg)  
        else:
            # republish the original message to output topic
            self.person_publisher.publish(in_msg)       

    def signal_handler(self, sig, frame):
        # Print statistics before quitting
        print("Summary Statistics:")
        # self.pr.print_stats()
        self.stats_publisher.publish(self.pr.print_stats())
        sys.exit(0)            


if __name__ == "__main__":
    try:
        rospy.init_node('unsw_person_recognition_node')
        pr = PersonRecogniserNode()        
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logfatal("Error occurred! Stopping the unsw person recognition node...")
            rospy.sleep(1)
            print("unsw_person_recognition_node terminated")
   

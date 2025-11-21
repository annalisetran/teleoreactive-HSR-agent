# Detection for Base Image Pipeline
# Represents detections in the image frame
Header header
sensor_msgs/Image image
# A segmented image with persons as foreground
sensor_msgs/Image person_seg_image
# List of objects detected in the frame
ObjectDetection[] objects
# List of People detected in the frame
PersonDetection[] people
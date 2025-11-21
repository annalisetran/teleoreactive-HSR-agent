#!/bin/bash

rosbag record \
    --output-prefix $1\
    /tf \
    /tf_static \
    /map \
    /map_markers \
    /spatial_visual_system/detections \
    /gpu/yolo2_objects_image/image_result \
    /gpu/Republisher/throttled_object \
    /gpu/Republisher/camera_info \
    /gpu/points \
    /yolo2_node/object_detections \
    /gpu/ObjectServicesNodelet/processed_detections \
    /hsrb/head_rgbd_sensor/depth_registered/camera_info \
    /hsrb/head_rgbd_sensor/ir/camera_info \
    /hsrb/head_rgbd_sensor/projector/camera_info \
    /hsrb/head_rgbd_sensor/rgb/camera_info \


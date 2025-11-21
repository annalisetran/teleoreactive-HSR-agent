#!/bin/bash

mkdir pointcloud_removed
rosbag filter $1 pointcloud_removed/$1.pcl_removed \
    "topic != '/gpu/points'"

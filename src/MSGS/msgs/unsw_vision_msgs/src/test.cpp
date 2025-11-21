/*
 * test.cpp
 *
 *  Created on: 04/07/2017
 *      Author: rescue
 */

#include <ros/ros.h>

#include <unsw_vision_msgs/Detection.h>
#include <unsw_vision_msgs/DetectionList.h>

#define LOG_START    "unsw_vision_msgs::Test ::"

// Actual main method outside of namespace
int main_vision_msgs(int argc, char** argv) {
    ros::init(argc, argv, "unsw_vision_msgs::test");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    return 0;
}



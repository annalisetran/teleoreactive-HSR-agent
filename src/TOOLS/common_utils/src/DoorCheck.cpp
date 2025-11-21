#include <utility/DoorCheck.h>
#include <iostream>
#include <string>

namespace utility {
    DoorCheck::DoorCheck(std::string frame_id, ros::NodeHandle *nh) {
        this->frame_id = frame_id;
        this->nh = nh;

        lastPose = NULL;
        lastScan = NULL;
        doorRadiusSearch = 2.5;

        doorsPub = nh->advertise<utility::DoorStatus>("/door_status", 100);
    }

    DoorCheck::~DoorCheck() {}

    void DoorCheck::setDoorRadiusSearch(float radius) {
    	this->doorRadiusSearch = radius;
    }


    void DoorCheck::poseCallback(const geometry_msgs::PoseStamped::ConstPtr&  pose){
        lastPose = pose;
    }


    void DoorCheck::laserCallback(const sensor_msgs::PointCloud2::ConstPtr& scan) {
        lastScan = scan;
    }

    void DoorCheck::checkDoors() {
        ROS_INFO("checking doors");
        if(lastPose == NULL || lastScan == NULL){
            return;
        }

        // get the transform between the pose frame and the map
        tf::StampedTransform poseTrans;
        try {
            tfListener.lookupTransform(frame_id, lastPose->header.frame_id, ros::Time(0), poseTrans);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("MapMarker::checkDoors pose-map: %s",ex.what());
            return;
        }

        // get the transform between the laser and the map
        tf::StampedTransform scanTrans;
        try {
            tfListener.lookupTransform(frame_id, lastScan->header.frame_id, ros::Time(0), scanTrans);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("MapMarker::checkDoors laser-map: %s",ex.what());
            return;
        }

        // get the pose of the robot in map coordinates
        tf::Pose mapPose;
        tf::poseMsgToTF(lastPose->pose, mapPose);
        mapPose = poseTrans*mapPose;

        // search is we are close to any door
        for (auto& element: doorList) {
            tf::Pose pose;
            tf::poseMsgToTF(element.second, pose);
            std::string name = element.first;

            // Calculate how close the robot is to any door
            tf::Pose relativePose = mapPose.inverse() * pose;
            relativePose.getOrigin().setZ(0.0);
            float distance = relativePose.getOrigin().length();
            //ROS_INFO("Door %s is %f far", name.c_str(), distance);

            // If the robot is not close enough we don't do anything
            if(distance <= doorRadiusSearch){
                pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg (*lastScan, *pclCloud);

                // Transform the pointcloud to locate the door on (0,0) coordinate
                tf::Transform projectionTransform = pose.inverse() * scanTrans;
                pcl_ros::transformPointCloud(*pclCloud, *cloudPointsProjected, projectionTransform);

                // Find if we can see the door from our position
                bool seeLeftFrame = false;
                bool seeRightFrame = false;
                pcl::CropBox<pcl::PointXYZ> boxFilter;
                // Check if can see any point around left side of frame
                boxFilter.setMin(Eigen::Vector4f(-0.3, -0.5, 0.0, 1.0));
                boxFilter.setMax(Eigen::Vector4f(0.3, -0.33, 1.0, 1.0));
                boxFilter.setInputCloud(cloudPointsProjected);
                boxFilter.filter(*cloudFiltered);
                if(cloudFiltered->size() > 0){
                    seeLeftFrame = true;
                }
                // Check if can see any point around right side of frame
                boxFilter.setMin(Eigen::Vector4f(-0.3, 0.33, 0.0, 1.0));
                boxFilter.setMax(Eigen::Vector4f(0.3, 0.5, 1.0, 1.0));
                boxFilter.setInputCloud(cloudPointsProjected);
                boxFilter.filter(*cloudFiltered);
                if(cloudFiltered->size() > 0){
                    seeRightFrame = true;
                }
                bool seeDoor = seeLeftFrame || seeRightFrame;
                bool fullView = seeLeftFrame && seeRightFrame;

                if(!seeDoor) {
                    return;
                }

                // Filter point cloud to find any points in the "door arsetDoorRadiusSearchea", if we find points the door is closed
                boxFilter.setMin(Eigen::Vector4f(-0.2, -0.30, 0.0, 1.0));
                boxFilter.setMax(Eigen::Vector4f(0.2, 0.30, 1.0, 1.0));
                boxFilter.setInputCloud(cloudPointsProjected);
                boxFilter.filter(*cloudFiltered);

                bool is_open = false;
                if(cloudFiltered->size() <= 5){ // Changed from 0 to 10 to account for some noise near the door
                    is_open = true;
                }

                DoorStatus msg;
                msg.name = name;
                msg.is_open = is_open;
                msg.view = fullView?utility::DoorStatus::FULL_VIEW:utility::DoorStatus::PARTIAL_VIEW;
                msg.position = element.second.position;
                doorsPub.publish(msg);
                ROS_INFO("I'm close to the door: %s %s %s, distance %f, see %d points", is_open?"OPEN":"CLOSED", fullView?"full view":"partial view", name.c_str(), distance, (int)cloudFiltered->size());
            }
        }

    }
}


/**
*/

#ifndef MAP_MARKERS_H

#define MAP_MARKERS_H

#include "ros/ros.h"

#include <utility/DoorStatus.h>
#include <utility/DoorList.h>
#include <utility/DBDoor.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <vector>
#include <map>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <algorithm>
#include <tf/tf.h>

namespace utility {

class DoorCheck {
    public:
        DoorCheck(std::string frame_id, ros::NodeHandle *nh);
        ~DoorCheck();
        void checkDoors();
        void setDoorRadiusSearch(float radius);
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
        void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& scan);

    private:
        std::map<std::string, geometry_msgs::Pose> doorList;
        ros::NodeHandle *nh;
        ros::Publisher doorsPub;
        ros::Subscriber doorsSub;
        std::string frame_id;
        geometry_msgs::PoseStamped::ConstPtr lastPose;
        sensor_msgs::PointCloud2::ConstPtr lastScan;
        tf::TransformListener tfListener;
        std::string name;
        bool is_open;
        bool view;
        geometry_msgs::Pose pose;

        float doorRadiusSearch;
};

}

#endif

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>

// #include <unsw_vision_msgs/BoundingBox.h>
// #include <unsw_vision_msgs/DetectionList.h>
// #include <unsw_vision_msgs/ObjectDetection.h>
// #include <geometry_msgs/Point32.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/subscriber.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <diagnostic_msgs/KeyValue.h>

#include<memory>

namespace vision {

    class PlaneDetector {
        public: 
            PlaneDetector(ros::NodeHandle &nh);
        private:
            // subscriber to the point cloud
            ros::Subscriber pointCloudSub;
            ros::Publisher planeMarkerPub;
            ros::Publisher planePCLPub;
            void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
            void publishPlaneMarker(const pcl::ModelCoefficients::Ptr& coefficients, 
                                  const std_msgs::Header& header,
                                  int plane_id);
            void publishPlaneInfo(const std::vector<pcl::ModelCoefficients::Ptr>& planes);
            ros::NodeHandle &nh_;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;

            // ROS parameters
            double planeDistThreshold_;
            int maxIterations_;
            bool optimizeCoefficients_;
            bool debug_; // for debugging and configuration. Turned off in actual task to save performance
            double plane_min_height_;
            double plane_max_height_;
            double maxAngleFromHorizontal_;
            double minPlaneSize_;
            int maxPlanes_;
            // debug and visualization
            ros::Publisher filteredPCLPub;
    };

}
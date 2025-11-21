#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <unsw_vision_msgs/BoundingBox.h>
#include <unsw_vision_msgs/DetectionList.h>
#include <unsw_vision_msgs/ObjectDetection.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <diagnostic_msgs/KeyValue.h>

#include<memory>

namespace vision {

    class PositionGenerator {
        public: 
            PositionGenerator(ros::NodeHandle &nh);
            geometry_msgs::PoseStamped getPosition(sensor_msgs::PointCloud2::ConstPtr pcl, unsw_vision_msgs::BoundingBox bbox, geometry_msgs::TransformStamped transform);
            int getsize(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::ObjectDetection obj, geometry_msgs::TransformStamped transform);
            _Float32 getDist(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::ObjectDetection obj, geometry_msgs::TransformStamped transform);
            _Float32 getDist(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::PersonDetection person, geometry_msgs::TransformStamped transform);
        private:
            void syncedCallback(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::DetectionList::ConstPtr detections);

            // transforming the point into the map frame

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;
            sensor_msgs::PointCloud2::ConstPtr pointCloud;
            std::string output_topic;
            std::string input_pcd_topic;
            std::string input_obj_topic;
            std::string source_frame_id;
            std::string target_frame_id;        
            ros::NodeHandle &nh_;
            ros::Publisher positionPub;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pointCloudSub;
            std::unique_ptr<message_filters::Subscriber<unsw_vision_msgs::DetectionList>> detectionSub;

            using ApproxPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, unsw_vision_msgs::DetectionList>;
            std::unique_ptr<message_filters::Synchronizer<ApproxPolicy>> syncApprox;
    };

}

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/subscriber.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <unsw_vision_msgs/DetectionList.h>

#include <cv_bridge/cv_bridge.h>

#include <memory>


class Republisher {
private:
  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>;
  using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2>;

  ros::NodeHandle& nh_;
  image_transport::ImageTransport it;
  cv::Mat lookUpTable;

  unsigned int sync_queue_size;
  unsigned int queue_size;
  bool use_exact;
  bool use_compressed;

  std::string input_rgb_topic;
  std::string input_pcd_topic;
  std::string output_rgb_topic;
  std::string output_pcd_topic; 

  // For subscrxibing to sensor data
  std::unique_ptr<image_transport::SubscriberFilter> sub_image_colour_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_;

  std::unique_ptr<image_transport::Publisher> rgbPub;
  std::unique_ptr<ros::Publisher> pcdPub;

  std::unique_ptr<message_filters::Synchronizer<ApproxPolicy>> syncApprox;
  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> syncExact;

  ros::Time prevFrame;
  int framerate;
  float timeFiltered;

public:
  Republisher(ros::NodeHandle& nh);
  void callback(sensor_msgs::Image::ConstPtr rgb, sensor_msgs::PointCloud2::ConstPtr pcd);
  void getParams(const std::string& path);
};

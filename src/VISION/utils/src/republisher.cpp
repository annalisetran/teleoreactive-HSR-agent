#include "republisher/republisher.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <chrono>
#include <iostream>
#include <filesystem>

Republisher::Republisher(ros::NodeHandle &nh) : nh_{nh}, it{nh}
{
  getParams("");
  image_transport::TransportHints hints(use_compressed ? "compressed" : "raw");

  sub_image_colour_ = std::make_unique<image_transport::SubscriberFilter>(it, input_rgb_topic, queue_size);
  sub_pcd_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, input_pcd_topic, queue_size);

  if (use_exact)
  {
    syncExact = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(sync_queue_size), *sub_image_colour_, *sub_pcd_
    );
    syncExact->registerCallback(boost::bind(&Republisher::callback, this, _1, _2));
  }
  else
  {
    syncApprox = std::make_unique<message_filters::Synchronizer<ApproxPolicy>>(
      ApproxPolicy(sync_queue_size), *sub_image_colour_, *sub_pcd_
    );
    syncApprox->registerCallback(boost::bind(&Republisher::callback, this, _1, _2));
    syncApprox->setMaxIntervalDuration(ros::Duration(10000));
  }
}

void Republisher::getParams(const std::string& path) {
  ROS_INFO("Republisher Parameters Set");
  nh_.getParam("republisher/input_rgb_topic", input_rgb_topic);
  nh_.getParam("republisher/input_pcd_topic", input_pcd_topic);
  nh_.getParam("republisher/output_rgb_topic", output_rgb_topic);
  nh_.getParam("republisher/output_pcd_topic", output_pcd_topic);  
  nh_.getParam("republisher/use_exact", use_exact);
  nh_.getParam("republisher/use_compressed", use_compressed);
  nh_.getParam("republisher/framerate", framerate);

  ROS_INFO("Input RGB: %s", input_rgb_topic.c_str());
  ROS_INFO("Input PCD: %s", input_pcd_topic.c_str());
  ROS_INFO("Input Use Exact: %s", use_exact ? "true" : "false");
  // nh_.getParam("republisher/rgb_topic", rgbTopic);
  // nh_.getParam("republisher/pcd_topic", pcdTopic);
  // nh_.getParam("republisher/use_exact", useExact);
  // nh_.getParam("republisher/use_compressed", useCompressed);
  // nh_.getParam("republisher/framerate", framerate);

  sync_queue_size = 20;
  queue_size = 20;
  lookUpTable = cv::Mat(1, 256, CV_8U);

  auto t0 = ros::Time::now();

  // Yolo
  prevFrame = t0;
  timeFiltered = 0;
  rgbPub = std::make_unique<image_transport::Publisher>(it.advertise(output_rgb_topic, 10));
  pcdPub = std::make_unique<ros::Publisher>(nh_.advertise<sensor_msgs::PointCloud2>(output_pcd_topic, 10));
  // rgbPub = std::make_unique<image_transport::Publisher>(it.advertise("republisher/rgb", 10));
  // pcdPub = std::make_unique<ros::Publisher>(nh_.advertise<sensor_msgs::PointCloud2>("republisher/pointcloud", 10));

}

void Republisher::callback(sensor_msgs::Image::ConstPtr rgb, sensor_msgs::PointCloud2::ConstPtr pcd)
{
  ros::Time now = ros::Time::now();

  // // gamma correction of the image to brighten the darks.
  // sensor_msgs::Image::ConstPtr imageModified;
  // try {
  //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);	
  //     cv::Mat res = cv_ptr->image.clone();
  //     LUT(cv_ptr->image, lookUpTable, res);
  //     cv_ptr->image = res;
  //     imageModified = cv_ptr->toImageMsg();
  // } catch (cv_bridge::Exception& e) {
  //     ROS_ERROR("cv_bridge exception: %s", e.what());
  //     imageModified = rgb;
  // }

  int nsecs = (now - prevFrame).toNSec();
  timeFiltered = timeFiltered * 0.9 + nsecs * 0.1;

  if( framerate > 0 && timeFiltered >= (1000000000.0/(framerate))) {
    rgbPub->publish(rgb);
    pcdPub->publish(pcd);
    prevFrame = now;
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "republisher");
    ros::NodeHandle nh;
    auto republisher = Republisher(nh);
    ros::spin();
    return 0;
}

#include <unsw_vision_utils/PlaneDetector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
namespace vision 
{
    PlaneDetector::PlaneDetector(ros::NodeHandle &nh) : nh_{nh}, tfListener(tfBuffer)
    {
        pointCloudSub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &PlaneDetector::pointCloudCallback, this);
        planeMarkerPub = nh.advertise<visualization_msgs::Marker>("detected_plane", 1);
        planePCLPub = nh.advertise<sensor_msgs::PointCloud2>("plane_pcl", 1);
        nh.param<double>("PlaneDetector/plane_distance_threshold", planeDistThreshold_, 0.01);
        nh.param<int>("PlaneDetector/max_iterations", maxIterations_, 100);
        nh.param<bool>("PlaneDetector/optimize_coefficients", optimizeCoefficients_, true);
        nh.param<bool>("PlaneDetector/debug", debug_, true);
        nh.param<double>("PlaneDetector/plane_min_height", plane_min_height_, 0.0);
        nh.param<double>("PlaneDetector/plane_max_height", plane_max_height_, 1.0);
        nh.param<double>("PlaneDetector/max_angle_from_horizontal", maxAngleFromHorizontal_, 0.1);
        nh.param<double>("PlaneDetector/min_plane_size", minPlaneSize_, 0.1);
        nh.param<int>("PlaneDetector/max_planes", maxPlanes_, 5);

        // debug pulishers
        if (debug_) {
            filteredPCLPub = nh.advertise<sensor_msgs::PointCloud2>("filterdPCL", 1);
            ROS_INFO("Plane Detector Debugging Enabled");
        } else {
            ROS_INFO("NOT DEBUGGING");
        }

    }
        void PlaneDetector::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
        {   
            // time
            time_t tstart;
            tstart = time(0);
            if (debug_) {
                nh_.getParam("PlaneDetector/plane_distance_threshold", planeDistThreshold_);
                nh_.getParam("PlaneDetector/max_iterations", maxIterations_);
                nh_.getParam("PlaneDetector/optimize_coefficients", optimizeCoefficients_);
                nh_.getParam("PlaneDetector/plane_min_height", plane_min_height_);
                nh_.getParam("PlaneDetector/plane_max_height", plane_max_height_);
                ROS_INFO("Plane Detector Parameters Set");
            }

            

        ROS_INFO("Received point cloud data with %d points", cloud_msg->width * cloud_msg->height);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud_in);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        std_msgs::Header transformed_header = cloud_msg->header;
        try {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("base_link", cloud_msg->header.frame_id, 
                cloud_msg->header.stamp, ros::Duration(1.0));
            pcl_ros::transformPointCloud(*cloud_in, *cloud_transformed, transformStamped.transform);
            transformed_header.frame_id = "base_link";
            ROS_INFO("Successfully transformed point cloud to base_link frame");
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_transformed);
        pass.setFilterFieldName("z");  // Filter along Z axis in base_link frame
        pass.setFilterLimits(plane_min_height_, plane_max_height_); // Keep points from ground level up to 3 meters
        pass.filter(*cloud_filtered);
        
        ROS_INFO("Filtered done at stamped: %f seconds", difftime(time(0), tstart));
        if (debug_) {
            // Publish the filtered point cloud for debugging
            sensor_msgs::PointCloud2 out_cloud_msg;
            pcl::toROSMsg(*cloud_filtered, out_cloud_msg);
            out_cloud_msg.header = transformed_header;
            filteredPCLPub.publish(out_cloud_msg);
        }

        if (cloud_filtered->points.size() < 100) {
            ROS_WARN("Not enough points after filtering (%lu points)", cloud_filtered->points.size());
            return;
        }
        
        // Working cloud that will progressively have planes removed
        pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *working_cloud = *cloud_filtered;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // Repeatedly find and remove planes until we've found enough or run out of points
        int plane_count = 0;
        while (plane_count < maxPlanes_ && working_cloud->points.size() > 100) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(optimizeCoefficients_);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(maxIterations_);
            seg.setDistanceThreshold(planeDistThreshold_);
            
            // Segment the largest planar component from the working cloud
            seg.setInputCloud(working_cloud);
            seg.segment(*inliers, *coefficients);
            
            if (inliers->indices.size() == 0) {
                ROS_INFO("No more planes found.");
                break;
            }
            
            // Check if the plane is approximately horizontal
            double a = coefficients->values[0];
            double b = coefficients->values[1];
            double c = coefficients->values[2];
            
            // Normalize normal vector
            double norm = sqrt(a*a + b*b + c*c);
            a /= norm;
            b /= norm;
            c /= norm;
            
            // Calculate angle between plane normal and vertical (0,0,1)
            double angle_deg = acos(fabs(c)) * 180.0 / M_PI;
            
            // Check if this is a horizontal plane based on our threshold
            if (angle_deg < maxAngleFromHorizontal_) {
                // Make sure the normal points up (positive z)
                if (c < 0) {
                    a = -a;
                    b = -b;
                    c = -c;
                    coefficients->values[0] = a;
                    coefficients->values[1] = b;
                    coefficients->values[2] = c;
                    coefficients->values[3] = -coefficients->values[3];
                }
                
                // Check plane size
                double plane_ratio = static_cast<double>(inliers->indices.size()) / cloud_filtered->points.size();
                if (plane_ratio >= minPlaneSize_) {
                    ROS_INFO("Found horizontal plane %d with %lu points (%.1f%% of cloud), angle: %.1f degrees",
                             plane_count+1, inliers->indices.size(), plane_ratio*100.0, angle_deg);
                    
                    // Extract points for this plane
                    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::ExtractIndices<pcl::PointXYZ> extract;
                    extract.setInputCloud(working_cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(false);
                    extract.filter(*plane_cloud);
                    
                    // Choose a color for this plane
                    uint8_t r, g, b;
                    switch (plane_count % 4) {
                        case 0: r = 0; g = 255; b = 0; break;    // Green
                        case 1: r = 0; g = 0; b = 255; break;    // Blue
                        case 2: r = 255; g = 0; b = 0; break;    // Red
                        case 3: r = 255; g = 255; b = 0; break;  // Yellow
                    }
                    
                    // Add colored points to the combined cloud
                    for (const auto& point : plane_cloud->points) {
                        pcl::PointXYZRGB colored_point;
                        colored_point.x = point.x;
                        colored_point.y = point.y;
                        colored_point.z = point.z;
                        colored_point.r = r;
                        colored_point.g = g;
                        colored_point.b = b;
                        all_planes_cloud->points.push_back(colored_point);
                    }
                    
                    // Visualize this plane with a marker
                    publishPlaneMarker(coefficients, transformed_header, plane_count);
                    
                    plane_count++;
                }
            }
            
            // Remove these points from the working cloud for the next iteration
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(working_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud_p);
            *working_cloud = *cloud_p;
        }
        
        // Publish the combined point cloud of all detected planes
        if (!all_planes_cloud->points.empty()) {
            all_planes_cloud->width = all_planes_cloud->points.size();
            all_planes_cloud->height = 1;
            all_planes_cloud->is_dense = false;
            
            sensor_msgs::PointCloud2 all_planes_msg;
            pcl::toROSMsg(*all_planes_cloud, all_planes_msg);
            all_planes_msg.header = transformed_header;
            planePCLPub.publish(all_planes_msg);
        } else {
            ROS_INFO("No suitable horizontal planes found.");
        }
        ROS_INFO("frame done at stamped: %f seconds", difftime(time(0), tstart));
    }

    void PlaneDetector::publishPlaneMarker(const pcl::ModelCoefficients::Ptr& coefficients, 
        const std_msgs::Header& header, int plane_id) {
        if (coefficients->values.size() != 4) {
        ROS_ERROR("Invalid plane coefficients");
        return;
        }

        // Create a marker to visualize the plane
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "plane_visualization";
        marker.id = plane_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Calculate normal vector
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];

        // Find a point on the plane
        double dist_to_origin = d; // d coefficient in ax + by + cz + d = 0

        // Center point of the marker - a point on the plane
        marker.pose.position.x = a * -dist_to_origin;
        marker.pose.position.y = b * -dist_to_origin;
        marker.pose.position.z = c * -dist_to_origin;

        // Set orientation - flat for horizontal planes
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker - adjust based on your environment
        marker.scale.x = 1.0;  // width
        marker.scale.y = 1.0;  // depth
        marker.scale.z = 0.01; // height (thin plate)

        // Set different colors for different planes
        switch (plane_id % 4) {
        case 0:
        marker.color.r = 0.0; marker.color.g = 0.8; marker.color.b = 0.0; // Green
        break;
        case 1:
        marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.8; // Blue
        break;
        case 2:
        marker.color.r = 0.8; marker.color.g = 0.0; marker.color.b = 0.0; // Red
        break;
        case 3:
        marker.color.r = 0.8; marker.color.g = 0.8; marker.color.b = 0.0; // Yellow
        break;
        }
        marker.color.a = 0.5; // Semi-transparent

        // Publish the marker
        planeMarkerPub.publish(marker);
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PlaneGenerator");
    ros::NodeHandle nh;
    vision::PlaneDetector node(nh);
    ros::spin();
    return 0;
}
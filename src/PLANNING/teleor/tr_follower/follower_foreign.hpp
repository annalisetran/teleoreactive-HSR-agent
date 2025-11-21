
#ifndef FOLLOWER_FOREIGN_HPP_
#define FOLLOWER_FOREIGN_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

//#include "QuProlog.h"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define FRONT		0
#define FRONT_LEFT	1
#define LEFT_FRONT	2
#define LEFT		3
#define LEFT_BACK	4
#define BACK_LEFT	5
#define BACK		6
#define BACK_RIGHT	7
#define RIGHT_BACK	8
#define RIGHT		9
#define RIGHT_FRONT	10
#define FRONT_RIGHT	11

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5


class FollowerNode : public rclcpp::Node
{
public:
	FollowerNode();
	~FollowerNode();
	bool near_start; 
	double scan_data_[12];

  void update_cmd_vel(double linear, double angular);
  
  
private:
	// ROS topic subscribers
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	// ROS topic publisher
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Variables
	double robot_pose_;
	double start_x, start_y;

	// Function prototypes
	void update_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};



#endif //FOLLOWER_FOREIGN_HPP_

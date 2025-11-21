
// We create a ROS node for gathering sensor data and for publishing
// drive commands. This node is run in it's own pthread so that it can collect
// sensor info asynchronously. Locks are used so that when the TR system
// asks for percepts we wait until sensor data update is not active.
//

#include "follower_foreign.hpp"
#include <unistd.h>
#include <memory>

#include <pthread.h>

using namespace std::chrono_literals;

// pthread management
pthread_t tid;
pthread_mutex_t p_lock;

// The percept gathering foreign function needs to access data in the
// follower node and the drive foreigh function needs to be able to call
// update_cmd_vel
std::shared_ptr<FollowerNode> current_node = NULL;

// This is based on Claude's sensor node except that an external timer is
// used and a foreign function extracts the current sensor data
FollowerNode::FollowerNode() : Node("tr_follower_node")
{
  /************************************************************
   ** Initialise variables
   ************************************************************/
  for (int i = 0; i < 12; i++)
    scan_data_[i] = 0.0;
  
  robot_pose_ = 0.0;
  near_start = false;
  
  
  /************************************************************
   ** Initialise ROS subscribers
   ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  
  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                   "scan", 
                   rclcpp::SensorDataQoS(),                             
                   std::bind(
                             &FollowerNode::scan_callback,               
                             this,                                      
                             std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
           "odom", qos, std::bind(&FollowerNode::odom_callback, this,
                                  std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  RCLCPP_INFO(this->get_logger(), "FollowerNode node has been initialised");
}

FollowerNode::~FollowerNode()
{
  current_node = NULL;
  RCLCPP_INFO(this->get_logger(), "FollowerNode node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
* Same as Claude's code except that thread locking is used
********************************************************************************/

#define START_RANGE	0.2

void FollowerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	pthread_mutex_lock(&p_lock); // Lock thread
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start!!\n");
		near_start = true;
		first = true;
		start_moving = true;
	}
	pthread_mutex_unlock(&p_lock); // Unlock thread
}

#define BEAM_WIDTH 10

void FollowerNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	pthread_mutex_lock(&p_lock); // Lock thread
	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
	pthread_mutex_unlock(&p_lock); // Unlock thread
}

void FollowerNode::update_cmd_vel(double linear, double angular)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "update_cmd_vel %fd %fd\n", linear, angular);
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}




// Code run in the follower thread
void *follower_node_thread(void *args){
  current_node = std::make_shared<FollowerNode>();
  rclcpp::spin(current_node);
  rclcpp::shutdown();
  return NULL;
}

// Foreign Function support


// Start up the FollowerNode in a pthread
extern "C" void *start_nodes(void)
{
  // Create lock for percept processing
  if (pthread_mutex_init(&p_lock, NULL) != 0) {
    printf("Mutex init failed\n");
    return NULL;
  }
  rclcpp::init(0, NULL);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Create FollowerNode thread\n");
  // run follower node in pthread
  pthread_create(&tid, NULL, follower_node_thread, NULL);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FollowerNode thread created\n");
  return NULL;
}

// the interface between TR actions and ROS actions 
extern "C" void drive(double linear, double angular)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drive %fd %fd\n", linear, angular);
  if (current_node == NULL) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drive NULL\n");
    sleep(2);
    if (current_node == NULL) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drive NULL Exit\n");
      return;
    }
  }
  current_node->update_cmd_vel(linear, angular);

}

// We only care about some of the sensor data - this returns
// the required data to the FF interface level
extern "C" bool get_percepts(bool *near, double *front, double *front_left,
			     double *left_front, double *front_right)
{
  if (current_node == NULL)
    return false;
  pthread_mutex_lock(&p_lock);
  *near = current_node -> near_start;
  *front = current_node -> scan_data_[0];
  *front_left = current_node -> scan_data_[1];
  *left_front = current_node -> scan_data_[2];
  *front_right = current_node -> scan_data_[11];
  
  pthread_mutex_unlock(&p_lock);
  return true;

}


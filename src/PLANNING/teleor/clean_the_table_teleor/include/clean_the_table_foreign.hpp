#ifndef CLEAN_THE_TABLE_FOREIGN_HPP
#define CLEAN_THE_TABLE_FOREIGN_HPP
// standard cpp includes
#include <vector>
#include <memory>
#include <array>
#include <string>

#include <unistd.h>
#include <cctype>

// ROS includes
#include "ros/ros.h"
#include <unsw_action_msg/UNSWActionMsg.h>
#include <std_msgs/String.h>

// Database includes
#include <pqxx/pqxx>

#define MAX_SCENE_OBJECTS 100
#define MAX_CLASS_NAME_LENGTH 64

struct SceneObject {
    int id;
    int class_id;
    std::array<double, 3> position;
    std::string class_name;
};

class CleanTheTableNode
{
public:
    CleanTheTableNode();
    ~CleanTheTableNode();

    // percepts
    // robot state percepts
    double base_pose_x;
    double base_pose_y;
    double base_pose_z;
    double base_pose_r;
    double base_pose_p;
    double base_pose_yw;
    bool driveable_state;
    int holding_object_id;
    int region_id;

    // scene object percept
    std::vector<SceneObject> scene_objects;

    // method to get percepts from database
    void get_percepts_from_database();

    // ROS actions
    void publish_goto(std::string region_name);
    void publish_goto(std::string frame_id, double x, double y, double z, double ox, double oy, double oz, double ow);
    void publish_pickup(std::string class_name, int object_id);
    void publish_place(std::string direction, std::string frame_id, double x, double y, double z);


private:
    pqxx::connection db_connection; // Database connection
    
    // ROS topic publisher
    ros::NodeHandle nh;
    ros::Publisher action_publisher;
};

extern pthread_t tid;
extern pthread_mutex_t p_lock;
void *clean_the_table_node_thread(void *args);

#endif // CLEAN_THE_TABLE_FOREIGN_HPP
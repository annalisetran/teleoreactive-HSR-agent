/*
SQL programs for getting sensor data
ROS node for publishing actions
*/

#include "clean_the_table_foreign.hpp"

pthread_t tid;
pthread_mutex_t p_lock;

// Global pointer to current node for extern C functions
std::shared_ptr<CleanTheTableNode> current_node = NULL;

CleanTheTableNode::CleanTheTableNode()
    : db_connection("dbname=world_model user=blinky")
{
    // Initialise member variables
    base_pose_x = 0;
    base_pose_y = 0;
    base_pose_z = 0;
    base_pose_r = 0;
    base_pose_p = 0;
    base_pose_yw = 0; 
    driveable_state = false;
    holding_object_id = -1;
    scene_objects.clear();

    action_publisher = nh.advertise<unsw_action_msg::UNSWActionMsg>("/actions", 10);

    ROS_INFO("CleanTheTableNode initialized and connected to database.\n");
}

CleanTheTableNode::~CleanTheTableNode()
{
    // Cleanup if necessary
    current_node = NULL;
    ROS_INFO("CleanTheTableNode has been terminated");
}

/***********************************************************************************************************
 * Database percept retrieval functions *****************************************************
 ***********************************************************************************************************/


void CleanTheTableNode::get_percepts_from_database()
{
    try
    {
        pqxx::work tx(db_connection);

        /*
        Get current robot state from robot_state table
        */

        pqxx::result robot_res = tx.exec(
            "SELECT base_pose_x, base_pose_y, base_pose_z,"
                "base_r, base_p, base_yw, driveable_state,"
                "holding_object_id, region_id "
            "FROM robot_state "
            "WHERE is_current = 1 "
            "ORDER BY created DESC "
            "LIMIT 1;"
        );
        
        if (!robot_res.empty()) {
            auto row = robot_res[0];
            base_pose_x = row["base_pose_x"].is_null() ? 0 : static_cast<double>(row["base_pose_x"].as<float>());
            base_pose_y = row["base_pose_y"].is_null() ? 0 : static_cast<double>(row["base_pose_y"].as<float>());
            base_pose_z = row["base_pose_z"].is_null() ? 0 : static_cast<double>(row["base_pose_z"].as<float>());
            base_pose_r = row["base_r"].is_null() ? 0 : static_cast<double>(row["base_r"].as<float>());
            base_pose_p = row["base_p"].is_null() ? 0 : static_cast<double>(row["base_p"].as<float>());
            base_pose_yw = row["base_yw"].is_null() ? 0 : static_cast<double>(row["base_yw"].as<float>());
            driveable_state = row["driveable_state"].is_null() ? false : row["driveable_state"].as<bool>();
            holding_object_id = row["holding_object_id"].is_null() ? -1 : row["holding_object_id"].as<int>();
            region_id = row["region_id"].is_null() ? -1 : row["region_id"].as<int>();
        } else {
            ROS_WARN("No current robot state found in the database.");
            base_pose_x = base_pose_y = base_pose_z = base_pose_r = base_pose_p = base_pose_yw = 0;
            driveable_state = false;
            holding_object_id = -1;
        }

        // Get current scene objects from v_cur_objects view
        scene_objects.clear();
        pqxx::result objects_res = tx.exec(
            "SELECT object_id, "
            "object_class_id, "
            "loc_x, "
            "loc_y, "
            "loc_z, "
            "last_updated, "
            "name AS object_class_name "
            "FROM v_cur_objects "
            "WHERE loc_x IS NOT NULL "
            "AND loc_y IS NOT NULL "
            "AND loc_z IS NOT NULL "
            "ORDER BY last_updated DESC;"
        );
        for (auto row : objects_res) {
            SceneObject obj;
            // Extract position coordinates
            obj.position = {
                static_cast<double>(row["loc_x"].as<float>()),
                static_cast<double>(row["loc_y"].as<float>()),
                static_cast<double>(row["loc_z"].as<float>())
            };

            obj.id = row["object_id"].as<int>();
            obj.class_id = row["object_class_id"].as<int>();
            obj.class_name = row["object_class_name"].as<std::string>();

            std::transform(obj.class_name.begin(), obj.class_name.end(), obj.class_name.begin(),
                [](unsigned char c) { return c == ' ' ? '_' : std::tolower(c); });

            this->scene_objects.push_back(obj);
        }

        tx.commit();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error getting percepts from database: %s", e.what());

        // Setting safe defaults in case of error
        base_pose_x = base_pose_y = base_pose_z = 0;
        base_pose_r = base_pose_p = base_pose_yw = 0;
        driveable_state = false;
        holding_object_id = -1;
        scene_objects.clear();
    
    }
}

/***********************************************************************************************************
 * ROS action publishing functions *************************************************************************
 ***********************************************************************************************************/
void CleanTheTableNode::publish_goto(std::string region_name)
{
    unsw_action_msg::UNSWActionMsg msg;
    msg.action_name = "goto_region";
    std::string json_data =  R"({"RegionName": ")" + region_name + R"("})";
    msg.data = json_data;
    action_publisher.publish(msg);
}

void CleanTheTableNode::publish_goto(std::string frame_id, double x, double y, double z, double ox, double oy, double oz, double ow)
{
    unsw_action_msg::UNSWActionMsg msg;
    msg.action_name = "goto_goal";
    std::string json_data = "{\"GoalPose\": {\"header\": {\"frame_id\": \"" + frame_id + 
                        "\"}, \"pose\": {\"position\": {\"x\": " + std::to_string(x) + 
                        ", \"y\": " + std::to_string(y) + ", \"z\": " + std::to_string(z) + 
                        "}, \"orientation\": {\"x\": " + std::to_string(ox) + 
                        ", \"y\": " + std::to_string(oy) + ", \"z\": " + std::to_string(oz) + 
                        ", \"w\": " + std::to_string(ow) + "}}}}";
    msg.data = json_data;
    action_publisher.publish(msg);
}

void CleanTheTableNode::publish_pickup(std::string object_class_name, int object_id)
{
    unsw_action_msg::UNSWActionMsg msg;
    msg.action_name = "manip_pickup";

    std::string json_data = "{\"class_name\": \"" + object_class_name + 
                           "\", \"object_id\": " + "\"" + std::to_string(object_id) + "\"" + 
                           ", \"direction\": \"front\"}";
    
    msg.data = json_data;
    action_publisher.publish(msg);
    
    ROS_INFO("Published pickup action: %s", json_data.c_str());
}

void CleanTheTableNode::publish_place(std::string direction, std::string frame_id, double x, double y, double z)
{
    unsw_action_msg::UNSWActionMsg msg;
    msg.action_name = "manip_placing";
    
    std::string json_data = "{\"direction\": \"" + direction + 
        "\", \"target_point\": {\"header\": {\"frame_id\": \"" + frame_id + 
        "\"}, \"point\": {\"x\": " + std::to_string(x) + 
        ", \"y\": " + std::to_string(y) + 
        ", \"z\": " + std::to_string(z) + "}}}";
    
    msg.data = json_data;
    action_publisher.publish(msg);
}


/***************************************************************************************************
extern "C" functions for C linkage to interface correctly with QuProlog ****************************
***************************************************************************************************/

void *clean_the_table_node_thread(void *args)
{
    current_node = std::make_shared<CleanTheTableNode>();
    ros::spin();
    ros::shutdown();
    return NULL;
}

// Start ROS node
extern "C" void *start_nodes(void)
{
    // Create lock for percept processing
    if (pthread_mutex_init(&p_lock, NULL) != 0) {
        printf("Mutex init failed\n");
        return NULL;
    }
    
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "clean_the_table_node");
    
    ROS_INFO("Creating CleanTheTableNode thread\n");
    // Run node in pthread
    pthread_create(&tid, NULL, clean_the_table_node_thread, NULL);
    ROS_INFO("CleanTheTableNode thread created\n");
    
    return NULL;
}


// Get robot state percepts helper function
extern "C" bool get_robot_state_percepts(double base_position[3], 
    double base_orientation[3], 
    bool *driveable_state, int *holding_object_id, int *region_id)
{
    if (current_node == nullptr) return false;

    base_position[0] = current_node->base_pose_x;
    base_position[1] = current_node->base_pose_y;
    base_position[2] = current_node->base_pose_z;
    base_orientation[0] = current_node->base_pose_r;
    base_orientation[1] = current_node->base_pose_p;
    base_orientation[2] = current_node->base_pose_yw;
    *driveable_state = current_node->driveable_state;
    *holding_object_id = current_node->holding_object_id;
    *region_id = current_node->region_id;

    return true;
}

// count is needed as length of scene_objects is unknown so size of C array cannot be determined
extern "C" bool get_object_count(int *count)
{
    if (current_node == nullptr) return false;

    *count = current_node->scene_objects.size();
    return true;
}

// Get individual scene objects by index helper function
extern "C" bool get_object_by_index(int index, int *object_id, int *class_id, double position[3], char class_name[MAX_CLASS_NAME_LENGTH])
{
    if (current_node == nullptr) return false;

    const std::vector<SceneObject>& objects = current_node->scene_objects;

    if (index < 0 || index >= static_cast<int>(objects.size())) {
        return false; // Index out of bounds
    }

    SceneObject obj = current_node->scene_objects[index];
    *object_id = obj.id;
    *class_id = obj.class_id;
    const std::array<double, 3>& pos = obj.position;
    position[0] = pos[0];
    position[1] = pos[1];
    position[2] = pos[2];

    strncpy(class_name, obj.class_name.c_str(), MAX_CLASS_NAME_LENGTH - 1);
    class_name[MAX_CLASS_NAME_LENGTH - 1] = '\0'; // Ensure null termination

    return true;
}

// Get objects by class name (case-insensitive) helper function
extern "C" bool get_objects_by_classname(const char *target_class_name,
                                                    int *object_ids, int *class_ids,
                                                    double (*positions)[3], char (*class_names)[MAX_CLASS_NAME_LENGTH],
                                                    int max_objects, int *found_count)
{
    if (current_node == nullptr) {
        ROS_ERROR("Current node is not initialized");
        return false;
    }
    
    if (target_class_name == nullptr) {
        ROS_ERROR("target_class_name is null");
        return false;
    }
    
    *found_count = 0;
    
    // Search through all scene objects
    for (size_t i = 0; i < current_node->scene_objects.size() && *found_count < max_objects; ++i) {
        const SceneObject& obj = current_node->scene_objects[i];
        
        // Compare class names (case-insensitive)
        if (strcasecmp(obj.class_name.c_str(), target_class_name) == 0) {
            // Found a match - add to output arrays
            object_ids[*found_count] = obj.id;
            class_ids[*found_count] = obj.class_id;
            
            positions[*found_count][0] = obj.position[0];
            positions[*found_count][1] = obj.position[1];
            positions[*found_count][2] = obj.position[2];
            
            strncpy(class_names[*found_count], obj.class_name.c_str(), MAX_CLASS_NAME_LENGTH - 1);
            class_names[*found_count][MAX_CLASS_NAME_LENGTH - 1] = '\0';
            
            (*found_count)++;
        }
    }
    
    ROS_DEBUG("Found %d objects of class '%s' (case-insensitive)", 
                *found_count, target_class_name);
    
    return true;
}

// Get all percepts in one call helper function
extern "C" bool get_percepts(double base_position[3], double base_orientation[3],
    bool *driveable_state, int *holding_object_id, int *region_id,
    int *object_ids, int *class_ids, 
    char (*class_names)[MAX_CLASS_NAME_LENGTH],
    double (*positions)[3], 
    int max_objects, int *object_count)
{
    if (current_node == nullptr) {
        ROS_ERROR("Current node is not initialized.\n");
        return false;
    }

    pthread_mutex_lock(&p_lock);
    //ROS_INFO("Getting percepts from database...\n");
    current_node->get_percepts_from_database();
    pthread_mutex_unlock(&p_lock);

    // get basic robot state percepts
    base_position[0] = current_node->base_pose_x;
    base_position[1] = current_node->base_pose_y;
    base_position[2] = current_node->base_pose_z;
    base_orientation[0] = current_node->base_pose_r;
    base_orientation[1] = current_node->base_pose_p;
    base_orientation[2] = current_node->base_pose_yw;
    *driveable_state = current_node->driveable_state;
    *holding_object_id = current_node->holding_object_id;
    *region_id = current_node->region_id;   

    size_t total_objects = current_node->scene_objects.size();
    *object_count = static_cast<int>(total_objects);

    if (*object_count > max_objects) {
        ROS_WARN("get_percepts: found %d objects but can only return %d (max_object limit)", 
                *object_count,
                max_objects
            );
            *object_count = max_objects;
    }

    for (int i = 0; i < *object_count; i++) {
        SceneObject obj = current_node->scene_objects[i];
        object_ids[i] = obj.id;
        class_ids[i] = obj.class_id;
        const std::array<double, 3>& pos = obj.position;
        positions[i][0] = pos[0];
        positions[i][1] = pos[1];
        positions[i][2] = pos[2];

    strncpy(class_names[i], obj.class_name.c_str(), MAX_CLASS_NAME_LENGTH - 1);
    class_names[i][MAX_CLASS_NAME_LENGTH - 1] = '\0';
    }

    ROS_DEBUG( "get_percepts: returned %d objects. Robot at (%f, %f, %f)", 
        *object_count, 
        base_position[0], 
        base_position[1], 
        base_position[2]
    );

    return true;
}

// Action functions

extern "C" void goto_action(char *region_name)
{
    if (current_node) {
        ROS_INFO("Publishing goto_action");
        current_node->publish_goto(std::string(region_name));
        ROS_INFO("goto_action published");
    }
}

extern "C" void goto_pose_action(char *frame_id, double x, double y, double z, double ox, double oy, double oz, double ow)
{
    if (current_node) {
        current_node->publish_goto(std::string(frame_id), x, y, z, ox, oy, oz, ow);
    }
}

extern "C" void pickup(char* class_name, int object_id)
{
    if (current_node) {
        current_node->publish_pickup(class_name, object_id);
    }
}

extern "C" void place(char *direction, char *frame_id, double x, double y, double z)
{
    if (current_node) {
        current_node->publish_place(std::string(direction), std::string(frame_id), x, y, z);
    }
}

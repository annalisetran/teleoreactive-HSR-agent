#include <utility/DoorCheck.h>

int main(int argc, char **argv) 
{   
    ros::init(argc, argv, "DoorCheck");
    ros::NodeHandle nh;
    ROS_INFO("Started");

    std::string frame_id;
    float radius;
    // nh.param<std::string>("/DoorCheck/frame_id", frame_id, "frame_id");
    // nh.param<float>("/DoorCheck/door_radius_search", radius, 0.5); // left this radious spelling unfixed to avoid conflicts
    ROS_INFO("Frame_id: %s", frame_id.c_str());

    utility::DoorCheck *door_check = new utility::DoorCheck(frame_id, &nh);
    door_check->setDoorRadiusSearch(1);
    
    // TODO: Add subscribers

    ros::Rate r(5);
    while (ros::ok())
    {   
        ROS_INFO("here");
        door_check->checkDoors();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
} 
#include <unsw_vision_utils/PositionGenerator.h>
#include <ctime>

/**
 * Generates 3d coordinates for each object in the detection list.
 * Also gives each object a size flag out of tiny, small, medium, large, extra large
*/

namespace vision 
{

    PositionGenerator::PositionGenerator(ros::NodeHandle &nh) : nh_{nh}, tfListener(tfBuffer){

        nh.param<std::string>("PositionGenerator/output_topic", output_topic, "/unsw_vision/detections/objects/positions");
        nh.param<std::string>("PositionGenerator/input_pcd_topic", input_pcd_topic, "/hsrb/head_rgbd_sensor/depth_registered/rectified_points");
        nh.param<std::string>("PositionGenerator/input_obj_topic", input_obj_topic, "/unsw_vision/detections/objects");
        nh.param<std::string>("PositionGenerator/source_frame_id", source_frame_id, "head_rgbd_sensor_link");
        nh.param<std::string>("PositionGenerator/target_frame_id", target_frame_id, "map");

        ROS_INFO("Position Generator Parameters Set");
        ROS_INFO("Input Object: %s", input_obj_topic.c_str());
        ROS_INFO("Input PCD: %s", input_pcd_topic.c_str());
        ROS_INFO("Output Topic: %s", output_topic.c_str());
        ROS_INFO("Source Frame: %s", source_frame_id.c_str());
        ROS_INFO("Target Frame: %s", target_frame_id.c_str());

        positionPub = nh.advertise<unsw_vision_msgs::DetectionList>(this->output_topic, 5);

        detectionSub = std::make_unique<message_filters::Subscriber<unsw_vision_msgs::DetectionList>>(nh, input_obj_topic, 5);
        pointCloudSub = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, input_pcd_topic, 20);
        // syncing
        syncApprox = std::make_unique<message_filters::Synchronizer<ApproxPolicy>>(
            ApproxPolicy(20), *pointCloudSub, *detectionSub
        );
        syncApprox->registerCallback(boost::bind(&PositionGenerator::syncedCallback, this, _1, _2));
        syncApprox->setMaxIntervalDuration(ros::Duration(5));
    }



    void PositionGenerator::syncedCallback(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::DetectionList::ConstPtr detections) {
        time_t tstart, tend;
        tstart = time(0);
        ROS_INFO("Position Generator - In callaback detection time - %d cloud time - %d", detections->header.stamp.sec, cloud->header.stamp.sec);
        unsw_vision_msgs::DetectionList newDetections;
        newDetections.header = detections->header;
        newDetections.image = detections->image;
        // Look up transform. This is done every callback and pass into the getPosition
        geometry_msgs::TransformStamped transform;
        try {
            transform = tfBuffer.lookupTransform(target_frame_id, source_frame_id, detections->header.stamp, ros::Duration(0.3));
        } catch (tf2::TransformException &ex) {
            // if lookup transform fails, return 
            ROS_INFO("PositionGenerator couldn't look up transform: %s", ex.what());
            return;
        }
        
        // setting positions of objects
        for (auto obj : detections->objects) {            
            geometry_msgs::PoseStamped pose = getPosition(cloud, obj.bbox,transform);
            obj.position.x = std::isnan(pose.pose.position.x) ? 0 : pose.pose.position.x;
            obj.position.y = std::isnan(pose.pose.position.y) ? 0 : pose.pose.position.y;
            obj.position.z = std::isnan(pose.pose.position.z) ? 0 : pose.pose.position.z;
            int mysize = getsize(cloud, obj,transform);
            obj.distance = getDist(cloud, obj, transform);
            diagnostic_msgs::KeyValue sizeKey;
            sizeKey.key = "size";
            sizeKey.value = std::to_string(mysize);
            obj.tags.push_back(sizeKey);
            //ROS_INFO("detected size");
            newDetections.objects.push_back(obj);
        }

        for (auto person : detections->people) {
            // setting position of person                        
            unsw_vision_msgs::BoundingBox personBbox = person.bbox_person;
            geometry_msgs::PoseStamped personPose = getPosition(cloud, personBbox, transform);
            person.position.x = std::isnan(personPose.pose.position.x) ? 0 : personPose.pose.position.x;
            person.position.y = std::isnan(personPose.pose.position.y) ? 0 : personPose.pose.position.y;
            person.position.z = std::isnan(personPose.pose.position.z) ? 0 : personPose.pose.position.z;

            geometry_msgs::PoseStamped personHeadPose = getPosition(cloud, person.bbox_head, transform);
            person.head_position.x = std::isnan(personHeadPose.pose.position.x) ? 0 : personHeadPose.pose.position.x;
            person.head_position.y = std::isnan(personHeadPose.pose.position.y) ? 0 : personHeadPose.pose.position.y;
            person.head_position.z = std::isnan(personHeadPose.pose.position.z) ? 0 : personHeadPose.pose.position.z;


            for (size_t i = 0; i < person.skeleton2d.size() && i < 17; ++i){
                unsw_vision_msgs::BoundingBox jointBox;
                auto &joint = person.skeleton2d[i];
                jointBox.cols = personBbox.cols;
                jointBox.rows = personBbox.rows;
                jointBox.height = 0;
                jointBox.width = 0;
                jointBox.x = joint.x;
                jointBox.y = joint.y;
                geometry_msgs::PoseStamped jointPose = getPosition(cloud, jointBox,transform);

                // joint.x = std::isnan(jointPose.pose.position.x) ? 0 : jointPose.pose.position.x;
                // joint.y = std::isnan(jointPose.pose.position.y) ? 0 : jointPose.pose.position.y;
                // joint.z = std::isnan(jointPose.pose.position.z) ? 0 : jointPose.pose.position.z;\

                geometry_msgs::Point32 newJoint;
                newJoint.x = std::isnan(jointPose.pose.position.x) ? 0 : jointPose.pose.position.x;
                newJoint.y = std::isnan(jointPose.pose.position.y) ? 0 : jointPose.pose.position.y;
                newJoint.z = std::isnan(jointPose.pose.position.z) ? 0 : jointPose.pose.position.z;
                person.skeleton[i] = newJoint;
            }
            person.distance =getDist(cloud,person,transform);
            newDetections.people.push_back(person);
            
            
        }
        newDetections.header.frame_id = target_frame_id;
        tend = time(0);
        ROS_INFO("It took %f seconds for pos generator to process a frame\n", difftime(tend, tstart));
        positionPub.publish(newDetections);
    }

    geometry_msgs::PoseStamped PositionGenerator::getPosition(sensor_msgs::PointCloud2::ConstPtr pcl, unsw_vision_msgs::BoundingBox bbox, geometry_msgs::TransformStamped transform) {
        
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = source_frame_id;
        pose.header.stamp = ros::Time::now();
        pose.header.seq = 0;
        
        pose.header = pcl->header; // check that this frame is head_rgbd_sensor_link// rescale the image coordinates to fit the point cloud resolution
        float scale = (float) bbox.cols / (float) pcl->width;
        bbox.x = (int) ((float) bbox.x / scale);
        bbox.y = (int) ((float) bbox.y / scale);
        bbox.height = (int) ((float) bbox.height / scale);
        bbox.width  = (int) ((float) bbox.width  / scale);
        // check boundary conditions one last time.
        if (bbox.x < 0) bbox.x = 0;
        if (bbox.y < 0) bbox.y = 0;
        if (bbox.x + bbox.width  >= pcl->width)  bbox.width  = pcl->width  - bbox.x - 1;
        if (bbox.y + bbox.height >= pcl->height) bbox.height = pcl->height - bbox.y - 1;

        int midPointx = (int) (bbox.x + ((float) bbox.width / 2));
        int midPointy = (int) (bbox.y + ((float) bbox.height / 2));

        _Float32 x;
        _Float32 y;
        _Float32 z;
        // _Float32 x = pcl->data[pcl->row_step * midPointy + pcl->point_step * midPointx];
        // _Float32 y = pcl->data[pcl->row_step * midPointy + pcl->point_step * midPointx + 4];
        // _Float32 z = pcl->data[pcl->row_step * midPointy + pcl->point_step * midPointx + 8];
        // copying point values from point_Float32 xtcloud to point
        memcpy(&x, &pcl->data[pcl->row_step * midPointy + pcl->point_step * midPointx], sizeof(_Float32));
        memcpy(&y, &pcl->data[pcl->row_step * midPointy + pcl->point_step * midPointx + 4], sizeof(_Float32));
        memcpy(&z, &pcl->data[pcl->row_step * midPointy + pcl->point_step * midPointx + 8], sizeof(_Float32));

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        try {
            tf2::doTransform(pose, pose, transform);
        } catch (tf2::TransformException &ex) {
            // if transform fails, return untransformed coordinates
            ROS_INFO("PositionGenerator couldn't find transform: %s", ex.what());
            return pose;
        }
        return pose;
    }

    int PositionGenerator::getsize(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::ObjectDetection obj, geometry_msgs::TransformStamped transform) {
        float angleIncrementY = 45 * (3.14159 / 180) / (cloud->data.size() / cloud->row_step);
        float angleIncrementX = 58 * (3.14159 / 180) / (cloud->row_step / cloud->point_step);
        geometry_msgs::PoseStamped cameraPose;
        cameraPose.pose.position.x = 0;
        cameraPose.pose.position.y = 0;
        cameraPose.pose.position.z = 0;

        try {
            tf2::doTransform(cameraPose, cameraPose, transform);
        } catch (tf2::TransformException &ex) {
            ROS_INFO("PositionGenerator couldn't find transform: %s", ex.what());
        }
        _Float32 relativeX = obj.position.x - cameraPose.pose.position.x;
        _Float32 relativeY = obj.position.y - cameraPose.pose.position.y;
        _Float32 relativeZ = obj.position.z - cameraPose.pose.position.z;
        _Float32 distance = sqrt(pow(relativeX, 2) + pow(relativeY, 2) + pow(relativeZ, 2));
        _Float32 height = distance * tan((obj.bbox.height / 2) * angleIncrementY) * 2;
        _Float32 width = distance * tan((obj.bbox.width / 2) * angleIncrementX) * 2;
        int sizeFlag = 1;
        if (height * width > 0.1 * 0.1 && height * width < 0.3 * 0.3) {
            sizeFlag = 2;
        } else if (height * width > 0.3 * 0.3 && height * width < 0.5 * 0.5) {
            sizeFlag = 3;
        } else if (height * width > 0.5 * 0.5 && height * width < 1.5 * 1.5) {
            sizeFlag = 4;
        } else {
            sizeFlag = 5;
        }
        return sizeFlag;

    }

    _Float32 PositionGenerator::getDist(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::ObjectDetection obj, geometry_msgs::TransformStamped transform) {
        float angleIncrementY = 45 * (3.14159 / 180) / (cloud->data.size() / cloud->row_step);
        float angleIncrementX = 58 * (3.14159 / 180) / (cloud->row_step / cloud->point_step);
        geometry_msgs::PoseStamped cameraPose;
        cameraPose.pose.position.x = 0;
        cameraPose.pose.position.y = 0;
        cameraPose.pose.position.z = 0;

        try {
            tf2::doTransform(cameraPose, cameraPose, transform);
        } catch (tf2::TransformException &ex) {
            ROS_INFO("PositionGenerator couldn't find transform: %s", ex.what());
            return -1.0;
        }

        _Float32 relativeX = obj.position.x - cameraPose.pose.position.x;
        _Float32 relativeY = obj.position.y - cameraPose.pose.position.y;
        _Float32 distance = sqrt(pow(relativeX, 2) + pow(relativeY, 2));
        return distance;
    }

    _Float32 PositionGenerator::getDist(sensor_msgs::PointCloud2::ConstPtr cloud, unsw_vision_msgs::PersonDetection person, geometry_msgs::TransformStamped transform) {
        float angleIncrementY = 45 * (3.14159 / 180) / (cloud->data.size() / cloud->row_step);
        float angleIncrementX = 58 * (3.14159 / 180) / (cloud->row_step / cloud->point_step);
        geometry_msgs::PoseStamped cameraPose;
        cameraPose.pose.position.x = 0;
        cameraPose.pose.position.y = 0;
        cameraPose.pose.position.z = 0;

        try {
            tf2::doTransform(cameraPose, cameraPose, transform);
        } catch (tf2::TransformException &ex) {
            ROS_INFO("PositionGenerator couldn't find transform: %s", ex.what());
            return -1.0;
        }

        _Float32 relativeX = person.position.x - cameraPose.pose.position.x;
        _Float32 relativeY = person.position.y - cameraPose.pose.position.y;
        _Float32 distance = sqrt(pow(relativeX, 2) + pow(relativeY, 2));
        return distance;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PositionGenerator");
    ros::NodeHandle nh;
    vision::PositionGenerator node(nh);
    ros::spin();
    return 0;
}
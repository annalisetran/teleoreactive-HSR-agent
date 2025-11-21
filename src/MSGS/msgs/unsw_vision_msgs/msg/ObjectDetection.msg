# Object Detection Information
string object_class
float32 object_class_conf

geometry_msgs/Point position   # 3d position
BoundingBox bbox        # 2d bbox
vision_msgs/BoundingBox3D bbox3d

# other attribute can be used in the tags
string KEY_SUBCLASS=object_subclass
string KEY_SUBCLASS_CONF=object_subclass_conf
string KEY_MOVABLE=movable
string KEY_SEAT_EMPTY=sittable
float32 distance   # distance to the object relative to baselink
int32 tracking_id
diagnostic_msgs/KeyValue[] tags
int32[] mask 


string id
string name
BoundingBox bbox_person                 # 2d bbox person class
BoundingBox bbox_head                   # 2d bbox head from head detection
vision_msgs/BoundingBox3D bbox3d        # of the person class
geometry_msgs/Point position            # 3d centroid of person
geometry_msgs/Point head_position       # 3d centroid of head (optional depending on where position is generated)
geometry_msgs/Point32[] face_encoding   
geometry_msgs/Point32[17] skeleton      # 3d keypoints in relation to the relevant frame (e.g. map)
geometry_msgs/Point32[17] skeleton2d    # 2d keypoints of pixel in relation to the original image

# Vector attach the the person
VisualVector left_pointing_vector
VisualVector right_pointing_vector
VisualVector gaze_vector


# Other attributes
string KEY_GENDER=gender
string KEY_HEIGHT=height
float32 distance   # distance to the person centroid relative to baselink
int32 tracking_id
diagnostic_msgs/KeyValue[] attributes # can change to json string

# Skeleton joints indices
#          4-2   1-3
#             \ /
#              0
#              |
#       6------x------5
#       |    |   |    |
#	    |    |   |    |
#       8    |   |    7
#       |    |   |    |
#       |    |   |    |
#       10   12  11   9
#            |   |
#            |   |
#            |   |
#           14   13
#            |   |
#            |   |
#            |   |
#           16   13
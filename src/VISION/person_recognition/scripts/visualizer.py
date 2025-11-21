
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from typing import List


def get_random_color(seed):
    gen = np.random.default_rng(seed)
    color = tuple(gen.choice(range(256), size=3))
    color = tuple([int(c) for c in color])
    return color


def print_objects_from_tags(objects):
    for obj in objects:
        print("List of Tags")
        for tag in obj.tags:
            obj_class=None
            class_conf=None
            if tag.key == "class" or tag.key == "class_conf":
                obj_class = tag.key
                class_conf = tag.value

            print("{} : {}".format(obj_class, class_conf))

def print_objects(objects):
    for obj in objects:
        print("Class= {}; Confidence={}".format(obj.object_class, obj.object_class_conf))

# img needs to be a numpy array
def draw_detections(img, objects):
    bridge = CvBridge()
    if not isinstance(img, np.ndarray):
        img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8") 

    for obj in objects:
        obj_class = obj.object_class
        class_conf = obj.object_class_conf

        color = get_random_color(int(abs(hash(obj_class)/100000)))
        img = cv2.rectangle(img, (int(obj.bbox.x), int(obj.bbox.y)), (int(obj.bbox.x)+int(obj.bbox.width), int(obj.bbox.y)+int(obj.bbox.height)), color, 2)
        
        # place the text at the top of the 
        x_text = int(obj.bbox.x)
        y_text = max(15, int(obj.bbox.y) + int(obj.bbox.height) - 10)

        img = cv2.putText(
            img, f'{obj_class}: {class_conf}', (x_text, y_text), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, color, 1, cv2.LINE_AA
        )

    return img


'''
im = original image
in_msg = DetectionList message that contains List of PersonDetections
'''
def plot_skeleton_kpts(im, in_msg):
    #Plot the skeleton and keypointsfor coco datatset
    palette = np.array([[255, 128, 0], [255, 153, 51], [255, 178, 102],
                        [230, 230, 0], [255, 153, 255], [153, 204, 255],
                        [255, 102, 255], [255, 51, 255], [102, 178, 255],
                        [51, 153, 255], [255, 153, 153], [255, 102, 102],
                        [255, 51, 51], [153, 255, 153], [102, 255, 102],
                        [51, 255, 51], [0, 255, 0], [0, 0, 255], [255, 0, 0],
                        [255, 255, 255]])

    skeleton = [[16, 14], [14, 12], [17, 15], [15, 13], [12, 13], [6, 12],
                [7, 13], [6, 7], [6, 8], [7, 9], [8, 10], [9, 11], [2, 3],
                [1, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7]]

    pose_limb_color = palette[[9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16]]
    pose_kpt_color = palette[[16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9]]
    radius = 5

    for person in in_msg:
        kpts = person.skeleton2d

        for kid in range(len(kpts)):
            r, g, b = pose_kpt_color[kid]

            x_coord, y_coord = kpts[kid].x, kpts[kid].y
            if not (x_coord % 640 == 0 or y_coord % 640 == 0):
                cv2.circle(im, (int(x_coord), int(y_coord)), radius, (int(r), int(g), int(b)), -1)

        for sk_id, sk in enumerate(skeleton):
            r, g, b = pose_limb_color[sk_id]
            pos1 = (int(kpts[sk[0]-1].x), int(kpts[sk[0]-1].y)) 
            pos2 = (int(kpts[sk[1]-1].x), int(kpts[sk[1]-1].y)) 

            if pos1[0] == 0 or pos1[1]==0 or pos1[0]<0 or pos1[1]<0:
                continue
            if pos2[0] == 0 or pos2[1] == 0 or pos2[0]<0 or pos2[1]<0:
                continue

            cv2.line(im, pos1, pos2, (int(r), int(g), int(b)), thickness=2)

    return im


def draw_face_bbox(img, face_list):
    # Display the results
    for face in face_list:            
        # Draw a box around the face
        cv2.rectangle(img, (int(face["bbox_head"].x), int(face["bbox_head"].y)), (int(face["bbox_head"].width), int(face["bbox_head"].height)), (0, 0, 255), 2)
        # Draw a label with a name below the face        
        cv2.rectangle(img, (int(face["bbox_head"].x), int(face["bbox_head"].height) - 35), (int(face["bbox_head"].width), int(face["bbox_head"].height)), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        #cv2.putText(img_frame, face["name"], (int(face["bb_left"]) + 6, int(face["bb_bottom"]) - 6), font, 1.0, (255, 255, 255), 1)
        cv2.putText(img, face["name"], (int(face["bbox_head"].x) + 6, int(face["bbox_head"].height) - 6), font, 1.0, (255, 255, 255), 1)

    return img

def draw_face_name(img, face_list):
    # Display the results
    for face in face_list:            
        # Draw a label with a name below the face
        font = cv2.FONT_HERSHEY_DUPLEX
        if face["bbox_head"] is not None:
            cv2.putText(img, face["name"], (int(face["bbox_head"].x) + 6, int(face["bbox_head"].height) - 6), font, 0.9, (0, 0, 255), 2)

    return img

def draw_face_info_from_msg(img, msg):
    font = cv2.FONT_HERSHEY_DUPLEX

    for person in msg.people:            
        # Draw a label with a name below the face        
        if person.bbox_head.width > 0:                    
            # cv2.putText(img, "Id: " + str(person.id), (int(person.bbox_head.x), int(person.bbox_head.y)), font, 0.9, (0, 0, 255), 1)
            cv2.putText(img, "Id: " + str(person.id) + ";" + str(person.tracking_id), (int(person.bbox_head.x), int(person.bbox_head.y)), font, 0.9, (0, 0, 255), 1)

    return img


'''
This method creates a window and displays the image with the faces and face information attached
'''
def display_image(self, img_frame, face_list):        
    img = draw_face_bbox(img_frame, face_list)
    # Display the resulting image
    cv2.imshow('Display Faces', img)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()                       

'''
This method republishes the image with the faces and face information attached
'''
def visualize_image(img, in_msg):
    bridge = CvBridge()
    if not isinstance(img, np.ndarray):
        img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8") 
        # add the header seq. number to the image
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(img, "Frame: " + str(in_msg.header.seq), (10, 470), font, 0.8, (0, 255, 0), 1)

    if len(in_msg.people) > 0:
        img = plot_skeleton_kpts(img, in_msg.people) 
        img = draw_face_info_from_msg(img, in_msg)        

    out_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    return out_msg
    
def print_agent(agent):
    if "pose" in agent:
        print("Id = {}; Name = {}; Pose = {}".format(agent["id"], agent["name"], agent["pose"]))
    else:
        print("Id = {}; Name = {};".format(agent["id"], agent["name"]))

def print_known_agents(known_agents):
    for agent in known_agents:
        print(agent)

def print_known_agents_old(known_agents):
    print("Known Agents:")
    for agent in known_agents:
        if "pose" in agent:
            print("Id = {}; Name = {}; Pose = {}".format(agent["id"], agent["name"], agent["pose"]))
        else:
            print("Id = {}; Name = {};".format(agent["id"], agent["name"]))


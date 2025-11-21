#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from unsw_vision_msgs.msg import PersonDetection, ObjectDetection

class Plotter():
    def __init__(self):
        pass       

        self.persons = []
        self.objects = []
        self.img = None

        rospy.loginfo("Plotter node ready")


    '''
    Function to reset the status of the plotter.
    Clear all the data that has been stored in the plotter.
    Ready for next image processing.
    Parameters:
    None
    Return:
    None
    '''
    def reset(self):
        pass
        self.persons = []
        self.objects = []
        self.img = None

    '''
    Function to add a person to the plotter for this frame.
    Parameters:
    person: PersonDetection
    Return:
    None
    '''
    def add_person(self, person: PersonDetection) -> None:
        pass
        self.persons.append(person)
    '''
    Function to add an object to the plotter for this frame.
    Parameters:
    obj: ObjectDetection
    Return:
    None
    '''
    def add_object(self, obj: ObjectDetection) -> None:
        pass
        self.objects.append(obj)

    '''
    Function to plot the image with the detected objects and persons.
    For a person detection, draw a bounding box around the person, with tracking id, class_name(person), skelton, head bounding box.
    For an object detection, draw a bounding box around the object, with tracking id, class_name, confidence score.
    Plot bbox in different colors for person and object.
    Parameters:
    img: cv2 image
    Return:
    plotted_img: cv2 image
    '''
    def plot(self, image: np.ndarray) -> np.ndarray:
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Plotting detected persons on image
        for person in self.persons:
        
            # Draw bounding boxes for person, and head on the image            
            cv2.rectangle(image, 
                          (int(person.bbox_person.x), int(person.bbox_person.y)),
                          (int(person.bbox_person.x + person.bbox_person.width), int(person.bbox_person.y + person.bbox_person.height)),
                          (255, 0, 0) if not person.bbox_person.occluded else (0,0,255), 2)
            cv2.putText(image,f"Person: {person.tracking_id}" ,(int(person.bbox_person.x), int(person.bbox_person.y)), font, 0.75,(255,255,255), 1)
            # cv2.putText(image,f"Person: {person.tracking_id}" ,(int(person.bbox_person.x), int(person.bbox_person.y)), font, 0.75,(255,255,255), 1, cv2.LINE_AA)
            # cv2.putText(image,f"distance: {round(person.distance,2)}" ,(int(person.bbox_person.x ), int(person.bbox_person.y+ person.bbox_person.height * 0.3)), font, 0.75,(255,255,255), 1, cv2.LINE_AA)
                       
            if (True):
                cv2.rectangle(image, 
                          (int(person.bbox_head.x), int(person.bbox_head.y)),
                          (int(person.bbox_head.x + person.bbox_head.width), int(person.bbox_head.y + person.bbox_head.height)),
                          (255,20,147) if not person.bbox_head.occluded else (0,0,255), 2)
            
            # Draw the skeleton2d

            # Head
            if (int(person.skeleton2d[3].x) != 0 and int(person.skeleton2d[3].x) != 0):
                if (int(person.skeleton2d[0].x) != 0 and int(person.skeleton2d[0].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[0].x), int(person.skeleton2d[0].y)), (int(person.skeleton2d[3].x), int(person.skeleton2d[3].y)), (255,20,147), 2)
                if (int(person.skeleton2d[1].x) != 0 and int(person.skeleton2d[1].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[1].x), int(person.skeleton2d[1].y)), (int(person.skeleton2d[3].x), int(person.skeleton2d[3].y)), (255,20,147), 2)
                if (int(person.skeleton2d[4].x) != 0 and int(person.skeleton2d[4].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[3].x), int(person.skeleton2d[3].y)), (int(person.skeleton2d[4].x), int(person.skeleton2d[4].y)), (255,20,147), 2)
                if (int(person.skeleton2d[5].x) != 0 and int(person.skeleton2d[5].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[3].x), int(person.skeleton2d[3].y)), (int(person.skeleton2d[5].x), int(person.skeleton2d[5].y)), (255,20,147), 2)
            
            if (int(person.skeleton2d[4].x) != 0 and int(person.skeleton2d[4].x) != 0):
                if (int(person.skeleton2d[2].x) != 0 and int(person.skeleton2d[2].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[4].x), int(person.skeleton2d[4].y)), (int(person.skeleton2d[2].x), int(person.skeleton2d[2].y)), (255,20,147), 2)
                if (int(person.skeleton2d[0].x) != 0 and int(person.skeleton2d[0].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[4].x), int(person.skeleton2d[4].y)), (int(person.skeleton2d[0].x), int(person.skeleton2d[0].y)), (255,20,147), 2)
                if (int(person.skeleton2d[6].x) != 0 and int(person.skeleton2d[6].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[4].x), int(person.skeleton2d[4].y)), (int(person.skeleton2d[6].x), int(person.skeleton2d[6].y)), (255,20,147), 2)
                
           
            # Torso
            if (int(person.skeleton2d[5].x) != 0 and int(person.skeleton2d[5].x) != 0):
                if (int(person.skeleton2d[6].x) != 0 and int(person.skeleton2d[6].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[5].x), int(person.skeleton2d[5].y)), (int(person.skeleton2d[6].x), int(person.skeleton2d[6].y)), (199,21,133), 2)
                if (int(person.skeleton2d[11].x) != 0 and int(person.skeleton2d[11].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[5].x), int(person.skeleton2d[5].y)), (int(person.skeleton2d[11].x), int(person.skeleton2d[11].y)), (199,21,133), 2)
            if (int(person.skeleton2d[6].x) != 0 and int(person.skeleton2d[6].x) != 0):
                if (int(person.skeleton2d[12].x) != 0 and int(person.skeleton2d[12].x) != 0):
                    cv2.line(image, (int(person.skeleton2d[6].x), int(person.skeleton2d[6].y)), (int(person.skeleton2d[12].x), int(person.skeleton2d[12].y)), (199,21,133), 2)
            if (int(person.skeleton2d[11].x) != 0 and int(person.skeleton2d[11].x) != 0 or int(person.skeleton2d[11].x) != 0 and int(person.skeleton2d[11].x) != 0):
                cv2.line(image, (int(person.skeleton2d[12].x), int(person.skeleton2d[12].y)), (int(person.skeleton2d[11].x), int(person.skeleton2d[11].y)), (199,21,133), 2)

            # Arms
            if (int(person.skeleton2d[6].x) != 0 and int(person.skeleton2d[6].x) != 0 or int(person.skeleton2d[8].x) != 0 and int(person.skeleton2d[8].x) != 0):             
                cv2.line(image, (int(person.skeleton2d[6].x), int(person.skeleton2d[6].y)), (int(person.skeleton2d[8].x), int(person.skeleton2d[8].y)), (255,105,180), 2)
            if (int(person.skeleton2d[8].x) != 0 and int(person.skeleton2d[8].x) != 0 or int(person.skeleton2d[10].x) != 0 and int(person.skeleton2d[10].x) != 0):             
                cv2.line(image, (int(person.skeleton2d[8].x), int(person.skeleton2d[8].y)), (int(person.skeleton2d[10].x), int(person.skeleton2d[10].y)), (255,105,180), 2)

            if (int(person.skeleton2d[5].x) != 0 and int(person.skeleton2d[5].x) != 0 or int(person.skeleton2d[7].x) != 0 and int(person.skeleton2d[7].x) != 0):
                cv2.line(image, (int(person.skeleton2d[5].x), int(person.skeleton2d[5].y)), (int(person.skeleton2d[7].x), int(person.skeleton2d[7].y)), (255,105,180), 2)
            if (int(person.skeleton2d[7].x) != 0 and int(person.skeleton2d[7].x) != 0 or int(person.skeleton2d[9].x) != 0 and int(person.skeleton2d[9].x) != 0):
                cv2.line(image, (int(person.skeleton2d[7].x), int(person.skeleton2d[7].y)), (int(person.skeleton2d[9].x), int(person.skeleton2d[9].y)), (255,105,180), 2)

            # Legs
            if (int(person.skeleton2d[12].x) != 0 and int(person.skeleton2d[12].x) != 0 or int(person.skeleton2d[14].x) != 0 and int(person.skeleton2d[14].x) != 0):             
                cv2.line(image, (int(person.skeleton2d[12].x), int(person.skeleton2d[12].y)), (int(person.skeleton2d[14].x), int(person.skeleton2d[14].y)), (219,112,147), 2)
            if (int(person.skeleton2d[14].x) != 0 and int(person.skeleton2d[14].x) != 0 or int(person.skeleton2d[16].x) != 0 and int(person.skeleton2d[16].x) != 0):
                cv2.line(image, (int(person.skeleton2d[14].x), int(person.skeleton2d[14].y)), (int(person.skeleton2d[16].x), int(person.skeleton2d[16].y)), (219,112,147), 2)

            if (int(person.skeleton2d[11].x) != 0 and int(person.skeleton2d[11].x) != 0 or int(person.skeleton2d[13].x) != 0 and int(person.skeleton2d[13].x) != 0):
                cv2.line(image, (int(person.skeleton2d[11].x), int(person.skeleton2d[11].y)), (int(person.skeleton2d[13].x), int(person.skeleton2d[13].y)), (219,112,147), 2)
            if (int(person.skeleton2d[13].x) != 0 and int(person.skeleton2d[13].x) != 0 or int(person.skeleton2d[15].x) != 0 and int(person.skeleton2d[15].x) != 0):             
                cv2.line(image, (int(person.skeleton2d[13].x), int(person.skeleton2d[13].y)), (int(person.skeleton2d[15].x), int(person.skeleton2d[15].y)), (219,112,147), 2)

        # Plotting detected objects on image
        for obj in self.objects:
            # Draw bounding box for the object
            cv2.rectangle(image, 
                          (int(obj.bbox.x), int(obj.bbox.y)),
                          (int(obj.bbox.x + obj.bbox.width), int(obj.bbox.y + obj.bbox.height)),
                          (255,69,0), 2)
            cv2.putText(image, f"{obj.object_class} {obj.object_class_conf:.1f}", (int(obj.bbox.x), int(obj.bbox.y)), font, 0.75,(255,255,255),1,cv2.LINE_AA)

        return image



    '''
    im = original image
    '''
    def plot_object_and_skeleton(self, img, plot_skeleton, plot_object):
        font = cv2.FONT_HERSHEY_SIMPLEX

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
        radius = 3

        for person in self.persons:
            # Draw bounding boxes for person, and head on the image
            x = int(person.bbox_person.x)
            y = int(person.bbox_person.y)
            w = int(person.bbox_person.width)
            h = int(person.bbox_person.height)

            # text position
            (text_w, text_h), _ = cv2.getTextSize(str(person.tracking_id), font, 0.5, 1)
            text_x = x + w - text_w - 5     # 5 pixels from the edge
            text_y = y + text_h + 5         # 5 pixels from the top

            cv2.rectangle(img, 
                (int(person.bbox_person.x), int(person.bbox_person.y)),
                (int(person.bbox_person.x + person.bbox_person.width), int(person.bbox_person.y + person.bbox_person.height)),
                (255, 0, 0) if not person.bbox_person.occluded else (0,0,255), 2)

            # cv2.rectangle(img, (x, y, (x + w), (y + h)), (255, 0, 0) if not person.bbox_person.occluded else (0,0,255), 2)
            # cv2.putText(img,f"{person.tracking_id}" ,(text_x, text_y), font, 0.75,(0,255,255), 1)            
            cv2.putText(img,f"Person: {person.tracking_id}" ,(int(person.bbox_person.x), int(person.bbox_person.y)), font, 0.75,(0,255,255), 1)    
            # cv2.putText(image,f"distance: {round(person.distance,2)}" ,(int(person.bbox_person.x ), int(person.bbox_person.y+ person.bbox_person.height * 0.3)), font, 0.75,(255,255,255), 1, cv2.LINE_AA)
                       
            if (True):
                cv2.rectangle(img, 
                          (int(person.bbox_head.x), int(person.bbox_head.y)),
                          (int(person.bbox_head.x + person.bbox_head.width), int(person.bbox_head.y + person.bbox_head.height)),
                          (255,20,147) if not person.bbox_head.occluded else (0,0,255), 2)

            if plot_skeleton:
                # Plot Skeleton
                kpts = person.skeleton2d

                for kid in range(len(kpts)):
                    r, g, b = pose_kpt_color[kid]

                    x_coord, y_coord = kpts[kid].x, kpts[kid].y
                    if not (x_coord % 640 == 0 or y_coord % 640 == 0):
                        cv2.circle(img, (int(x_coord), int(y_coord)), radius, (int(r), int(g), int(b)), -1)

                for sk_id, sk in enumerate(skeleton):
                    r, g, b = pose_limb_color[sk_id]
                    pos1 = (int(kpts[sk[0]-1].x), int(kpts[sk[0]-1].y)) 
                    pos2 = (int(kpts[sk[1]-1].x), int(kpts[sk[1]-1].y)) 

                    if pos1[0] == 0 or pos1[1]==0 or pos1[0]<0 or pos1[1]<0:
                        continue
                    if pos2[0] == 0 or pos2[1] == 0 or pos2[0]<0 or pos2[1]<0:
                        continue

                    cv2.line(img, pos1, pos2, (int(r), int(g), int(b)), thickness=1)

        if plot_object:
            # Plotting detected objects on image
            for obj in self.objects:
                # Draw bounding box for the object
                cv2.rectangle(img, 
                            (int(obj.bbox.x), int(obj.bbox.y)),
                            (int(obj.bbox.x + obj.bbox.width), int(obj.bbox.y + obj.bbox.height)),
                            (255,69,0), 2)
                cv2.putText(img, f"{obj.object_class} {obj.object_class_conf:.1f}", (int(obj.bbox.x), int(obj.bbox.y)), font, 0.75,(0,255,255),1,cv2.LINE_AA)

        return img

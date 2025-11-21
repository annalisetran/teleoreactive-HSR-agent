#!/usr/bin/env python3

# Scene Utilities
# Helper functions to assist with scene processing and object tracking

def check_bbox_imgborder(bbox,img_w, img_h):
    touching_border = bbox.x == 0 or bbox.y == 0 or bbox.x + bbox.width == img_w or bbox.y + bbox.height == img_h

    return touching_border

def check_bbox_overlap(bbox1, bbox2):
    x_min1 = bbox1.x
    y_min1 = bbox1.y
    x_max1 = bbox1.x+bbox1.width
    y_max1 = bbox1.y+bbox1.height
    x_min2 = bbox2.x
    y_min2 = bbox2.y
    x_max2 = bbox2.x+bbox2.width
    y_max2 = bbox2.y+bbox2.height

    horizontal_overlap = (x_max1 >= x_min2) and (x_max2 >= x_min1)
    vertical_overlap = (y_max1 >= y_min2) and (y_max2 >= y_min1)

    return horizontal_overlap and vertical_overlap 

def check_bbox_nexto(bbox1, bbox2):
    x_min1 = bbox1.x
    y_min1 = bbox1.y
    x_max1 = bbox1.x+bbox1.width
    y_max1 = bbox1.y+bbox1.height
    x_min2 = bbox2.x
    y_min2 = bbox2.y
    x_max2 = bbox2.x+bbox2.width
    y_max2 = bbox2.y+bbox2.height

    if ((x_min1 <= x_max2 and x_max1 >= x_min2) and (y_min1 <= y_max2 and y_max1 >= y_min2) ):
        return True
    else:
        return False
    
def calculate_overlap_area(bbox1, bbox2):
    x_min1 = bbox1.x
    y_min1 = bbox1.y
    x_max1 = bbox1.x+bbox1.width
    y_max1 = bbox1.y+bbox1.height
    x_min2 = bbox2.x
    y_min2 = bbox2.y
    x_max2 = bbox2.x+bbox2.width
    y_max2 = bbox2.y+bbox2.height

    #claculate the coordinates of the intersecting rectangle
    x_left = max(x_min1, x_min2)
    y_top = max(y_min1, y_min2)
    x_right = min(x_max1, x_max2)
    y_bottom = min(y_max1, y_max2)

    width = max(0, x_right - x_left)
    height = max(0,y_bottom - y_top)

    overlap_area = width * height

    return overlap_area


def check_occlusion(test_obj, obj_list, img_width, img_height):
    for obj in obj_list:
        if test_obj != obj:
            border = check_bbox_imgborder(test_obj.bbox, img_width, img_height)
            next_to = check_bbox_nexto(test_obj.bbox, obj.bbox)
            #overlap = check_bbox_overlap(test_obj.bbox, obj.bbox)

            if border:
                # print(f"Occ Type: Border; Test Obj.Class: {test_obj.object_class}; Dist: {test_obj.distance}")
                return True
            # check the distance between the objects to work out which object is occluded   
            overlap_area = calculate_overlap_area(test_obj.bbox, obj.bbox)         
            if next_to and overlap_area < 250 and test_obj.distance > obj.distance:                
                # print(f"Occ Type: Next To; Test Obj.Class: {test_obj.object_class}; Dist: {test_obj.distance} -> Comp Obj.Class: {obj.object_class}; Dist: {obj.distance}; Overlap Area: {overlap_area}")
                return True
                
    return False


# Understanding the State Machine

## Start
- This state uses the database to determine wrist press
- Transitions `DOOR_CHECK` after wrist press

## DOOR CHECK
- This state subscribes to `/door_open_close` to determine door opened
- Transitions to`MOVE_TO_CABINET_A` after door detected open

## MOVE TO CABINET

### TODO
- Currently this state uses the cabinet pose values from clean the table -> These must be updated for the competition

### Other
- Currently there are two instances of this state `MOVE_TO_CABINET_A` and `MOVE_TO_CABINET_B`
- `MOVE_TO_CABINET_A` transitions to `OPEN_CABINET_DOOR_A`, `MOVE_TO_CABINET_B` transitions to `LOOK_AT_CABINET`

## OPEN CABINET DOOR

### TODO
- Add door opening behaviour
  - Currently this node asks for human assistance and sleeps for 5 seconds

### Other
- There are two different states `OpenCabinetDoorA` and `OpenCabinetDoorB`, there only difference is the spoken statement
- As there would likely be difference between the behaviours for each door, the two states act as placeholders


## DEFINE SHELVES

### TODO
- Define the first and second viewing states of the cabinet
- Test the plane detection
- Update `object_category_dict` -> this is a dict with the key as the object_class and value as the category

### Method
- Plane detection callback -> `plane_marker_dict` that stores markers by their id (plane detection max id is 5)

1. The robot moves into the first viewing pose
  - Uses `wait_for_message` to get a `DetectionList`
  - Stores a copy of `plane_marker_dict`
2. The robot moves into the second viewing pose
  - Uses `wait_for_message` to get a `DetectionList`
  - Gets another copy of `plane_marker_dict`
3. Create new combined plane dict
  - Combines plane dicts (as a list of tuples) and sorts ascending by z-value of marker
  - Make list back into dict (`shelf_dict`) based on their index in the list
4. Add categories (as defined in `object_category_dict`) of Object Detections to `shelf_dict` based on the marker closest below their z-value
5. Create a python set `categories_set` of all the object categories currently in the shelves


## MOVE TO TABLE
### TODO
- Define a pose for table (for viewing table specifically)

## VIEW TABLE
### TODO
- Define joint states for manipulation off table
### Logic
- After moving to the manip view pose the node will loop through all objects in ObjectDetections
- For each it will find the object category as defined in `object_category_dict`
- If the object category is in `categories_set`, will then add the detected object to `ud.pickup`
  - Also uses talker to communicate object class and object category
  - Also uses ObjectFrame servuce to display image of the object
- If there is no object chosen by the first method, it will then find the first object that has an `object_class` in `object_category_dict`

## ATTEMPT APPROACH
### TODO
- Define the approach pose for each object class in `self.approach_pose`

## HAND CENTERING
- This is an empty state that was carried over from clean the table

## ATTEMPT GRASP
- This state was carried over from clean the table, transitions to `RETRACT_ARM`

## RETRACT ARM
- This state was carried over from clean the table, transitions to `MOVE_TO_CABINET_B`

## LOOK AT CABINET

### TODO
- Define manip view pose goal for cabinet

### Transitions
- This transitions to `PLACE_FAIL` if the object in `ud.pickup` is not in the `object_category_dict` or `ud.categories_set`
- It transitions to `APPROACH_PLACE` otherwise and moves into manip view pose

## APPROACH PLACE
- This is an empty state that was carried over from clean the table

## PLACE OBJECT

### Logic
- Currently this node finds the best shelf by looping through `current_cabinet` until a shelf contains the object category
- It then uses the position of the shelf marker (z-value + 0.05) as the `target_point` for manip placing

## RETRACT ARM PLACE
- This state was carried over from clean the table, transitions to `MOVE_TO_TABLE`

## PLACE FAIL
- This state currently states that it couldn't find a shelf to match the object category and asks for assistance
- It then `rospy.sleep`s for 10 seconds before transitioning to `MOVE_TO_TABLE`

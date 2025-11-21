# ManipApproach Action Server Documentation

## Action Name

`manip_approach`

## Action Definition

### Goal

- `geometry_msgs/PointStamped target_point`: The target point for the robot to approach.
- `string direction`: The direction of approach (valid options: "top", "front", "front-vertical", updated 10/10/2024).

### Result

- `bool action_success`: Indicates whether the action was successful.
- `float64 forward_distance`: Indicate how far the grasp is away from the target object. This can be used for grasp action.

### Feedback

- `uint32 feedback`: Indicates the current status of the action.
  - 0: Not started
  - 1: Ongoing
  - 2: Failure

## Available Topics

1. `manip_appraoch/goal` (subscribed by the server)

   - Used to send goals to the action server.
2. `manip_appraoch/cancel` (subscribed by the server)

   - Used to cancel an active goal.
3. `manip_appraoch/feedback` (published by the server)

   - Provides feedback on the action's progress.
4. `manip_appraoch/result` (published by the server)

   - Publishes the final result of the action.
5. `manip_appraoch/status` (published by the server)

   - Publishes the current status of the action server.



## Usage Notes

- The action server uses the HSR (Human Support Robot) interface for movement and manipulation.
- It supports approaching objects from the top, front, or front-vertical directions.
- The gripper is opened before approaching the target.
- If the action fails (e.g., due to invalid parameters or movement failures), it will be aborted, and the result will indicate failure.

# ManipGrasp Action Server Documentation

## Action Name

`manip_grasp`

## Action Definition

### Goal

- `float64 forward_distance`: Indicate how far the grasp is moving forward then close

### Result

- `bool action_success`: Indicates whether the action was successful.

### Feedback

- `uint32 feedback`: Indicates the current status of the action.
  - 0: Not started
  - 1: Ongoing
  - 2: Failure

## Available Topics

1. `manip_grasp/goal` (subscribed by the server)

   - Used to send goals to the action server.
2. `manip_grasp/cancel` (subscribed by the server)

   - Used to cancel an active goal.
3. `manip_grasp/feedback` (published by the server)

   - Provides feedback on the action's progress.
4. `manip_grasp/result` (published by the server)

   - Publishes the final result of the action.
5. `manip_grasp/status` (published by the server)

   - Publishes the current status of the action server.



## Usage Notes

- The action server uses the HSR (Human Support Robot) interface for movement and manipulation.
- The gripper is opened before trying to grasp
- If the action fails (e.g., due to invalid parameters or movement failures), it will be aborted, and the result will indicate failure.

# ManipRetract Action Server Documentation

## Action Name

`manip_retract`

## Action Definition

### Goal

### Result

- `bool action_success`: Indicates whether the action was successful.

### Feedback

- `uint32 feedback`: Indicates the current status of the action.
  - 0: Not started
  - 1: Ongoing
  - 2: Failure

## Available Topics

1. `manip_retract/goal` (subscribed by the server)

   - Used to send goals to the action server.
2. `manip_retract/cancel` (subscribed by the server)

   - Used to cancel an active goal.
3. `manip_retract/feedback` (published by the server)

   - Provides feedback on the action's progress.
4. `manip_retract/result` (published by the server)

   - Publishes the final result of the action.
5. `manip_retract/status` (published by the server)

   - Publishes the current status of the action server.



## Usage Notes

- The action server uses the HSR (Human Support Robot) interface for movement and manipulation.
- This action simply retracts the arm to carry pose.
- If the action fails (e.g., due to invalid parameters or movement failures), it will be aborted, and the result will indicate failure.

# ManipPlacing Action Server Documentation

## Action Name

`manip_placing`

## Action Definition

### Goal

- `geometry_msgs/PointStamped target_point`: The target point for the robot to approach.
- `string direction`: The direction of approach (valid options: "top", "front", "front-vertical", updated 10/10/2024).

### Result

- `bool action_success`: Indicates whether the action was successful.

### Feedback

- `uint32 feedback`: Indicates the current status of the action.
  - 0: Not started
  - 1: Ongoing
  - 2: Failure

## Available Topics

1. `manip_placing/goal` (subscribed by the server)

   - Used to send goals to the action server.
2. `manip_placing/cancel` (subscribed by the server)

   - Used to cancel an active goal.
3. `manip_placing/feedback` (published by the server)

   - Provides feedback on the action's progress.
4. `manip_placing/result` (published by the server)

   - Publishes the final result of the action.
5. `manip_placing/status` (published by the server)

   - Publishes the current status of the action server.



## Usage Notes

- The action server uses the HSR (Human Support Robot) interface for movement and manipulation.
- This action place a object currently holding onto a surface
- If the action fails (e.g., due to invalid parameters or movement failures), it will be aborted, and the result will indicate failure.





## Action Name

`manip_gaze_at'

## Action Definition

### Goal

- `geometry_msgs/PointStamped target_point`: The target point for the robot to look at.

### Result

- `bool action_success`: Indicates whether the action was successful.

### Feedback

- `string feedback`: Indicates the current status of the action. A json string of all joints values in a dictionary format.
e.g """{"head_pan_joint": 1.3}"""

## Available Topics

1. `manip_gaze_at/goal` (subscribed by the server)

   - Used to send goals to the action server.
2. `manip_gaze_at/cancel` (subscribed by the server)

   - Used to cancel an active goal.
3. `manip_gaze_at/feedback` (published by the server)

   - Provides feedback on the action's progress.
4. `manip_gaze_at/result` (published by the server)

   - Publishes the final result of the action.
5. `manip_gaze_at/status` (published by the server)

   - Publishes the current status of the action server.



## Usage Notes

- The action server uses the HSR (Human Support Robot) interface for movement and manipulation.
- This action move the head and look at a point
- This action will move the base so that the right side of robot face the target point for minimising obstruction! BE EXTREMELY CAREFUL WITH THE WIRES DURING TESTING!
- If the action fails (e.g., due to invalid parameters or movement failures), it will be aborted, and the result will indicate failure.




## Action Name

`manip_view_pose'

## Action Definition

### Goal

- `float32 arm_lift_joint`: joint value for arm lift.
- `float32 arm_flex_joint`: joint value for arm flex joint.
- `float32 head_tilt_joint`: joint value for head tilt joint.
- `float32 head_pan_joint`: joint value for head pan joint.

### Result

- `bool action_success`: Indicates whether the action was successful.

### Feedback

- `uint32 feedback`: Indicates the current status of the action.
  - 0: Not started
  - 1: Ongoing
  - 2: Failure

## Available Topics

1. `manip_view_pose/goal` (subscribed by the server)

   - Used to send goals to the action server.
2. `manip_view_pose/cancel` (subscribed by the server)

   - Used to cancel an active goal.
3. `manip_view_pose/feedback` (published by the server)

   - Provides feedback on the action's progress.
4. `manip_view_pose/result` (published by the server)

   - Publishes the final result of the action.
5. `manip_view_pose/status` (published by the server)

   - Publishes the current status of the action server.



## Usage Notes

- The action server uses the HSR (Human Support Robot) interface for movement and manipulation.
- This action move the head and arm to specific joints value to be able to view items on table surfaces
- If the action fails (e.g., due to invalid parameters or movement failures), it will be aborted, and the result will indicate failure.

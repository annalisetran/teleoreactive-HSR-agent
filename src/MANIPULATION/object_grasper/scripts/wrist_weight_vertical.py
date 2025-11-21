#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
import hsrb_interface

FORCE_X_OFFSET = - 2.6  # Adjust this offset based on your calibration
FORCE_Y_OFFSET = - 4.55  # Adjust this offset based on your calibration
FORCE_Z_OFFSET = 38.27  # Adjust this offset based on your calibration
class ForceSensorCapture(object):
    """Subscribe and hold force sensor data with averaging"""
    __BUFFER_SIZE = 100

    def __init__(self, ft_sensor_topic):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        self._past_readings = []
        self._next_buf_index = 0
        self._buf_full = False

        # Subscribe force torque sensor data from HSRB
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb, queue_size=2)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def clear_buffer(self):
        self._next_buf_index = 0
        self._past_readings = []
        self._buf_full = False

    def __ft_sensor_cb(self, data):
        force_x = data.wrench.force.x + FORCE_X_OFFSET
        force_y = data.wrench.force.y + FORCE_Y_OFFSET
        force_z = data.wrench.force.z + FORCE_Z_OFFSET

        self.__push_to_buf([force_x, force_y, force_z])

        num_readings = len(self._past_readings)
        self._force_data_x = sum(
            [x[0] for x in self._past_readings]) / num_readings
        self._force_data_y = sum(
            [x[1] for x in self._past_readings]) / num_readings
        self._force_data_z = sum(
            [x[2] for x in self._past_readings]) / num_readings

    def __push_to_buf(self, reading):
        if not self._buf_full:
            self._past_readings.append(reading)
            self._next_buf_index += 1
            if self._next_buf_index >= self.__BUFFER_SIZE:
                self._buf_full = True
                self._next_buf_index = 0
        else:
            self._past_readings[self._next_buf_index] = reading
            self._next_buf_index = (
                self._next_buf_index + 1) % self.__BUFFER_SIZE


class ForceChangeDetector:
    def __init__(self):

        rospy.init_node('force_change_detector_node')
        
        # Parameters
        self.wrist_frame = rospy.get_param('~wrist_frame', 'wrist_ft_sensor_frame')
        self.world_frame = rospy.get_param('~world_frame', 'base_footprint')
        self.gap_indicates_counter = 5
        
        # Thresholds for object detection
        self.weight_threshold = rospy.get_param('~weight_threshold', 10)  # Newtons
        self.gripper_gap_threshold = rospy.get_param('~gripper_gap_threshold', 0.003)  # meters
        
        self.force_calculator = ForceSensorCapture("/hsrb/wrist_wrench/raw")
        self.tf_listener = tf.TransformListener()
        
        # Initialize robot interface
        try:
            self._robot = hsrb_interface.Robot()
            self._gripper = self._robot.get('gripper', self._robot.Items.END_EFFECTOR)
        except Exception as e:
            self._robot = None
            self._gripper = None
        
        # Gripper state tracking
        self.gripper_state = "unknown"  # "opening", "closing"
        self.gripper_distance = 0.0
        self.hand_motor_velocity = 0.0
        self.is_holding_object = False
        
        # Subscribe to joint states for gripper motion detection
        self.joint_state_sub = rospy.Subscriber(
            '/hsrb/joint_states', JointState, self.joint_state_callback, queue_size=10)
        
        # Create publishers
        self.force_pub = rospy.Publisher('/gravity_aligned_force', Float64, queue_size=10)
        self.holding_object_pub = rospy.Publisher('/holding_object', Bool, queue_size=10)
        
        # Timer for publishing force and object detection at regular intervals
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_data)

    def joint_state_callback(self, msg):
        """Callback for joint state messages to track gripper motion"""
        try:
            # Get hand motor joint velocity and position directly
            if 'hand_motor_joint' in msg.name:
                joint_index = msg.name.index('hand_motor_joint')
                
                # Get velocity
                self.hand_motor_velocity = msg.velocity[joint_index]
                
                # Update gripper state based on velocity
                if self.hand_motor_velocity < -1.0:
                    self.gripper_state = "closing"
                elif self.hand_motor_velocity > 1.0:
                    self.gripper_state = "opening"
                # No stopped state - keep previous state when velocity is low
                    
                # Get joint position (can be used for additional calculations if needed)
                joint_position = msg.position[joint_index]
                    
        except (ValueError, IndexError) as e:
            pass

    def get_gripper_distance_safe(self):
        """Safely get gripper distance with error handling"""
        try:
            if self._gripper is not None:
                return self._gripper.get_distance()
            else:
                return 0.0
        except Exception as e:
            return 0.0

    def get_gravity_vector_in_wrist_frame(self):
        """
        Transform the gravity vector (pointing up in world frame) 
        to the wrist force sensor frame
        """
        try:
            # Create a unit vector pointing up in the world frame
            gravity_vector = Vector3Stamped()
            gravity_vector.header.frame_id = self.world_frame
            gravity_vector.header.stamp = rospy.Time(0)
            gravity_vector.vector.x = 0.0
            gravity_vector.vector.y = 0.0
            gravity_vector.vector.z = 1.0  # Unit vector pointing up
            
            # Wait for transform to be available
            self.tf_listener.waitForTransform(
                self.wrist_frame, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
            
            # Transform gravity vector to wrist frame
            wrist_gravity_vector = self.tf_listener.transformVector3(
                self.wrist_frame, gravity_vector)
            
            # Normalise transformed vector
            v = wrist_gravity_vector.vector
            magnitude = np.sqrt(v.x**2 + v.y**2 + v.z**2)
            
            return [v.x/magnitude, v.y/magnitude, v.z/magnitude]
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # Return default orientation if TF fails
            rospy.logwarn(f"TF Exception: {e}. Returning default gravity vector.")
            return [0.0, 0.0, 1.0]

    def get_gravity_aligned_force(self):
        """
        Get force component along gravity direction
        
        Returns:
            float: Force in Newtons along gravity direction
        """
        # Get current force readings from the wrist sensor
        force_vector = self.force_calculator.get_current_force()
        
        # Get gravity direction in wrist frame
        gravity_dir = self.get_gravity_vector_in_wrist_frame()
        
        # DO NOT DELETE. FOR CALIBRATION PURPOSES
        # Compute dot product to get force component along gravity direction
        # print(f"x force_vector[0] * gravity_dir[0]: {force_vector[0]} * {gravity_dir[0]}")
        # print(f"y force_vector[1] * gravity_dir[1]: {force_vector[1]} * {gravity_dir[1]}")
        # print(f"z force_vector[2] * gravity_dir[2]: {force_vector[2]} * {gravity_dir[2]}")
        force_magnitude = -(force_vector[0] * gravity_dir[0] + 
                           force_vector[1] * gravity_dir[1] + 
                           force_vector[2] * gravity_dir[2])
        print(f"Force aligned with gravity: {force_magnitude} N")
        return force_magnitude

    def detect_holding_object(self):
        """
        Detect if the robot is holding an object based on:
        1. Weight (gravity-aligned force)
        2. Gripper gap when state is closed
        
        Returns:
            bool: True if holding an object
        """
        # Get current measurements
        gravity_force = self.get_gravity_aligned_force()
        gripper_distance = self.get_gripper_distance_safe()
        
        # Store for debugging
        self.gripper_distance = gripper_distance
        
        # Check weight condition
        weight_indicates_object = abs(gravity_force) > self.weight_threshold
        
        # Check gripper gap condition 
        # decrement self.gap_indicates_counter if gripper is closing and gap is larger than threshold
        if self.gripper_state == "closing" and gripper_distance > self.gripper_gap_threshold:
            self.gap_indicates_counter = max(0, self.gap_indicates_counter - 1)
        else:
            self.gap_indicates_counter = 5
        
        # Object is detected if either condition is met
        holding_object = weight_indicates_object or self.gap_indicates_counter == 0
        
        self.is_holding_object = holding_object
        return holding_object

    def publish_data(self, event):
        """Timer callback to publish force estimates and object detection"""
        try:
            # Get force aligned with gravity
            force = self.get_gravity_aligned_force()
            
            # Detect if holding object
            holding_object = self.detect_holding_object()
            
            # Publish force
            self.force_pub.publish(Float64(force))
            
            # Publish object holding status
            self.holding_object_pub.publish(Bool(holding_object))
            
        except Exception as e:
            pass

if __name__ == '__main__':
    try:
        # Create and run the force detector
        detector = ForceChangeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
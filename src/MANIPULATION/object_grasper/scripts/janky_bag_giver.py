#!/usr/bin/env python3

import sys
import rospy
import actionlib
import geometry_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import tf
import tf.transformations
from math import pi, sqrt
import hsrb_interface
from hsrb_interface import geometry
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped, PointStamped, Twist, PoseStamped
from sensor_msgs.msg import JointState
from time import sleep
from tmc_msgs.msg import Voice

_CONNECTION_TIMEOUT = 10.0

def ros_msg_to_quat(msg):
    return [msg.x, msg.y, msg.z, msg.w]

class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""
    __BUFFER_SIZE = 100

    def __init__(self, ft_sensor_topic):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        self._past_readings = []
        self._next_buf_index = 0
        self._buf_full = False

        self._ft_sensor_topic = ft_sensor_topic

        # Subscribe force torque sensor data from HSRB
        self._wrist_wrench_sub = rospy.Subscriber(
            self._ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)

        # # Wait for connection
        # try:
        #     rospy.wait_for_message(self._ft_sensor_topic, WrenchStamped,
        #                            timeout=_CONNECTION_TIMEOUT)
        # except Exception as e:
        #     rospy.logerr(e)
        #     sys.exit(1)

    def get_current_force(self):
        rospy.loginfo("get_current_force: <{}, {}, {}>"
                      .format(self._force_data_x, self._force_data_y, self._force_data_z))
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def clear_buffer(self):
        self._next_buf_index = 0
        self._past_readings = []
        self._buf_full = False

    def __ft_sensor_cb(self, data):
        force_x = data.wrench.force.x
        force_y = data.wrench.force.y
        force_z = data.wrench.force.z

        self.__push_to_buf([force_x, force_y, force_z])

        numReadings = len(self._past_readings)
        self._force_data_x = sum(
            [x[0] for x in self._past_readings]) / numReadings
        self._force_data_y = sum(
            [x[1] for x in self._past_readings]) / numReadings
        self._force_data_z = sum(
            [x[2] for x in self._past_readings]) / numReadings

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

class luggageGiver(object):
    def __init__(self):
        #rospy.init_node('luggage_giver_node')
        
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self._list_controllers = rospy.ServiceProxy("hsrb/controller_manager/list_controllers",
                                                    ListControllers)
        self._transformer = tf.TransformListener()
        self._forceCalculator = ForceSensorCapture("/hsrb/wrist_wrench/raw")
        self._omni_base_client = actionlib.SimpleActionClient(
                "/hsrb/omni_base_controller/follow_joint_trajectory",
                control_msgs.msg.FollowJointTrajectoryAction)
        self._output_speech = rospy.Publisher('/talk_request', Voice, queue_size = 10)
        self._arm_controller_client = actionlib.SimpleActionClient(
            "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
            control_msgs.msg.FollowJointTrajectoryAction)
        
        
        self.controller_subscriber = rospy.Subscriber('/object_grasper/bag/release_start', String, self.place_bag)
        
        self.timeout = 30
    
    
    def move_base_rel(self, x, y, yaw, duration=10):
        try:
            self._list_controllers.wait_for_service(5)
        except Exception as e:
            rospy.logerr("list controllers service not available")
            return False

        # Check controller is running
        controllers = self._list_controllers().controller
        filter(lambda c: c.name == "omni_base_controller", controllers)
        # Only be one left...
        for c in controllers:
            if c.state != "running":
                rospy.logerr("omni_base_controller not up!")
                return False
    
        # follow joint trajectory server takes odom points
        # We need to transform our goal...
        base_goal_pose = PoseStamped()
        base_goal_pose.header.stamp = rospy.Time.now()
        base_goal_pose.header.frame_id = "base_link"

        base_goal_pose.pose.position.x = x
        base_goal_pose.pose.position.y = y
        base_goal_pose.pose.position.z = 0

        rot_numpy = tf.transformations.quaternion_from_euler(0, 0, yaw)
        base_goal_pose.pose.orientation.x = rot_numpy[0]
        base_goal_pose.pose.orientation.y = rot_numpy[1]
        base_goal_pose.pose.orientation.z = rot_numpy[2]
        base_goal_pose.pose.orientation.w = rot_numpy[3]


        odom_goal_pose = self.transform_pose(base_goal_pose, "odom")

        (_, _, yaw_odom) = tf.transformations.euler_from_quaternion(
                ros_msg_to_quat(odom_goal_pose.pose.orientation))

        #lookup_time = rospy.Time.now()
        #self._transformer.waitForTransform("base_link", "odom", lookup_time, rospy.Duration(5))
        #(base_to_odom_pos, base_to_odom_rot) = self._transformer.lookupTransform("odom", "base_link", lookup_time)

        #yaw_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #odom_yaw_quat = base_to_odom_rot * yaw_quat

        #(_, _, yaw_odom) = tf.transformations.euler_from_quaternion(odom_yaw_quat)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()

        p.positions = [odom_goal_pose.pose.position.x,
                odom_goal_pose.pose.position.y,
                yaw_odom]

        p.velocities = [0, 0, 0]
        p.time_from_start = rospy.Time(duration)
        traj.points = [p]
        goal.trajectory = traj

        self._omni_base_client.send_goal(goal)

        completed = self._omni_base_client.wait_for_result()

        if not completed:
            return False
        
        result = self._omni_base_client.get_result()

        if not result.error_code == result.SUCCESSFUL:
            rospy.logerr("ManipulationController failed moving base. Error {}"
                    .format(result.error_string))
            return False

        return True

    def transform_pose(self, pose, target_frame):
        self._transformer.waitForTransform(
            pose.header.frame_id, target_frame, pose.header.stamp, rospy.Duration(5))
        return self._transformer.transformPose(target_frame, pose)       
    
    def open_gripper(self):
        self._gripper.command(1.2)
        
    def close_gripper(self):
        self._gripper.command(-0.047)   

    def check_current_weight(self, axis=(1,1,1)):
        # Averages the last 10 readings
        # Note we assume that nothing is pulling forwards on the hand etc.
        # So we can just take the length of the force vector as being the sum of the weights of the hand and object
        f = self._forceCalculator.get_current_force()

        # Multiply the f and axis vectors
        f_axis = [x*y for (x, y) in zip(f, axis)]
        # print(f)
        # fTotal = math.sqrt(sum([x*x for x in f]))
        # Convert newtowns to grams
        # We assume that the hand is horizontal
        # TODO: Do a proper transform!
        
        fTotal = sqrt(sum([x*x for x in f_axis]))
        weight = round(fTotal / 9.81, 1)
        return weight
    
    def say(self, text):
        # if self._enable_speech:
        self._output_speech.publish(False, False, 1, text)
        # else:
        #     rospy.loginfo("Say: %s", text) 

    def move_to_give(self):
        jointValues = {
            'arm_flex_joint': 0,
            'arm_lift_joint': 0,
            'arm_roll_joint': 0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': -1.57
        }
        self._whole_body.move_to_joint_positions(jointValues)

    def place_bag(self):
        time = 0
        start_weight = self.check_current_weight((1,1,0))
        while abs(start_weight - weight) > 10 or time > self.timeout:
            self.move_to_give()

            self.open_gripper()

            weight = self.check_current_weight((1,1,0))

            self.say("Here is your luggage. Please take your bag. Have a nice day!")

            sleep(5)
            time += 0
            
        self._whole_body.move_to_go()
        
        
def main():
    try: 
        commander = luggageGiver()
        
        # print ("============ Press `Enter` to execute a base movement using a pose goal ...")
        # input()
        # commander.move_base_rel(1,1,0)

        input ("============ Press `Enter` to release bag ...")
        commander.place_bag()

        print ("============ Placing complete!")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__=='__main__':
    rospy.init_node("bag_giver_node")
    luggageGiver()
    rospy.spin()
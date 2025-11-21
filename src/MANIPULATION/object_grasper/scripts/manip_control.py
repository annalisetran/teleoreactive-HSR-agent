#!usr/bin/env python3

import sys
import rospy
import actionlib
import geometry_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
from math import pi, sqrt, radians
import hsrb_interface
from hsrb_interface import geometry
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped, PointStamped, Twist, PoseStamped
from sensor_msgs.msg import JointState
from time import sleep
from tmc_msgs.msg import Voice
import tf2_ros
import tf2_geometry_msgs
from hsrb_interface import collision_world
from manip_srv.srv import hand_plane_move, wrist_rotate, approach_object, grasp_object, retract_arm,placing_object, receive_pose, give_pose, view_pose
import sys
#sys.path.append('/home/robocupathome/workspace/temp_ws/src/object_grasper/scripts')
#sys.path.insert(0, '/home/robocupathome/workspace/temp_ws/src/object_grasper/scripts')

#from myRobot import MyRobot


_CONNECTION_TIMEOUT = 5.0

PALM_SIZE = 0.06

class Robot():
    def __init__(self) -> None:
        self.robot = hsrb_interface.Robot()
        
        self._whole_body = self.robot.get('whole_body')
        self._whole_body.linear_weight = 100.0
        self._whole_body.angular_weight = 100.0
        
        self._gripper= self.robot.get('gripper')
        


        return
        # set up collision world
        cw = self.robot.get("global_collision_world")
        
        # Don't remove the line, it's necessary to wait for the collision world to be ready. Can adjust this to be a busy waiting loop later on

        # I remove the line, replaced with a busy waiting loop
        
        while(not rospy.is_shutdown()):
            if cw.add_box(x=0, y=0, z=0, pose=geometry.pose(x=0, y=0, z=10000), frame_id='map') is not None:
                print("init collision world done")
                cw.remove_all()
                break
            else:
                print("waiting for collision world")
                
        # attach the table
        TABLE_HEIGHT = 0.74
        TABLE_THICKNESS = 0.1
        #cw.add_box(x=0.74, y=1.18, z=0.74, pose=geometry.pose(x=-5.009, y=2.341, z=0.37,ek = radians(-40)), frame_id='map')
        
        #cw.add_box(x=0.74, y=1.18, z=TABLE_THICKNESS, pose=geometry.pose(x=-5.009, y=2.341, z=TABLE_HEIGHT-TABLE_THICKNESS/2,ek = radians(-35)), frame_id='map')
        #rospy.loginfo("table added")
        self._whole_body.collision_world = cw
        #print(cw.add_box(x=5, y=5, z=10, pose=geometry.pose(x=0.4, y=0, z=0.2), frame_id='base_link'))
        
        #print(cw.add_box(x=5, y=5, z=10, pose=geometry.pose(x=0.4, y=0, z=0.2), frame_id='base_link'))
        #print(cw.add_sphere(radius=20, pose=geometry.pose(x=20.0, y=1.0, z=0.5),frame_id='base_link'))
    
        #self._whole_body.collision_world = cw
        
        # board = self._whole_body.collision_world.add_box(x=0.74, y=1.18, z=0.74, pose=geometry.pose(x=-5.73, y=-1.95, z=0.74), frame_id='map')
    def get_whole_body(self):
        return self._whole_body
            
    def get_gripper(self):
        return self._gripper


class luggageGrasper():
    def __init__(self):
        rospy.loginfo("grasper starting")
        self._robot = Robot()
        #self._whole_body = self._robot.get_whole_body()
        # self._gripper = self._robot.get_gripper()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(2)
        self._output_speech = rospy.Publisher('/talk_request', Voice, queue_size = 10)
        self._arm_controller_client = actionlib.SimpleActionClient(
            "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
            control_msgs.msg.FollowJointTrajectoryAction)
        # self.point_subscriber = rospy.Subscriber('/object_grasper/bag/handle_coordinate', PointStamped,self.grabBagCB)
        self.point_subscriber = rospy.Subscriber('/pickup_point2', PointStamped,self.grabBagCB)
        self.point_subscriber = rospy.Subscriber('/pickup_point', PointStamped,self.graspFront)
        self.gaze_subscriber = rospy.Subscriber('/gaze_point', PointStamped,self.gazeCb, queue_size=1)
        rospy.Subscriber("/debug",String, self.debug)
        rospy.Subscriber("/test_hand_impedance",String, self.test_hand_impedance)
        
        
        self.manip_publisher = rospy.Publisher('/object_grasper/bag/pickup_status', String, queue_size=10)
        self.init_pose_sub = rospy.Subscriber('/init_pose', String, self.task_init_pose)
        self.open_gripper_sub = rospy.Subscriber("/open_grasp", String, self.openGraspCB)

        
        self.timeout = 15
        rospy.loginfo("init node")
        # 10N is the pressure when the hand is empty and in init pose
        self.wrist_pressure = 10
        self.empty_wrist_pressure = 10
        self.grip_pressure = 0
        self.wrist_pressure_sub = rospy.Subscriber("hsrb/wrist_wrench/raw", WrenchStamped, self.wristCB)
        s = rospy.Service('/unsw/manip/hand_plane_move', hand_plane_move, self.end_effector_plane_move)
        wrist_rotate_s = rospy.Service('/unsw/manip/wrist_rotate', wrist_rotate, self.wrist_rotate)
        rospy.Service('unsw/manip/approach_object',approach_object , self.approach_object )
        rospy.Service('unsw/manip/grasp_object',grasp_object , self.grasp_object )
        rospy.Service('unsw/manip/retract_arm',retract_arm , self.retract_arm )
        rospy.Service('unsw/manip/placing_down',placing_object , self.placing_down )
        rospy.Service('unsw/manip/placing_down/table',placing_object , self.placing_down_table )
        rospy.Service('unsw/manip/receive_pose', receive_pose, self.receive_pose_for_bag)
        rospy.Service('unsw/manip/give_pose', give_pose, self.give_pose_for_bag)
        rospy.Service('unsw/manip/view_pose', view_pose, self.change_to_view_pose)


    def wristCB(self, data):
      self.wrist_pressure = data.wrench.force.x
      # pressure on the wrist when grip opens vertically
      self.grip_pressure = data.wrench.force.y
          
    
    def move_to_carry_pose(self):
        jointValues = {
            'arm_flex_joint': -0.13,
            'arm_lift_joint': 0.15,
            'arm_roll_joint': 2.28,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0
        }
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
        
    def move_to_receive_pose(self):
        jointValues = {
            'arm_flex_joint': 0,
            'arm_lift_joint': 0,
            'arm_roll_joint': 0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': -1.57
        }
        rospy.loginfo("moving to receive pose")
        self._robot.get_whole_body().move_to_joint_positions(jointValues)

    
    def task_init_pose(self, data):
        rospy.loginfo("init pose")
        self._robot.get_whole_body().move_to_neutral()
        jointValues = {
                'arm_flex_joint': 0,
                'arm_lift_joint': 0,
                'arm_roll_joint': -1.58,
                'wrist_flex_joint': -1.57,
                'wrist_roll_joint': 0,
                }
        self._robot.get_gripper().command(0.0)
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
    
    def move_to_armup_pose(self):
        rospy.loginfo("init pose")
        jointValues = {
                'arm_flex_joint': -1.56,
                'arm_lift_joint': 0.52,
                'arm_roll_joint': 0,
                'wrist_flex_joint': -1.47,
                'wrist_roll_joint': -0.2,
                }
        self._robot.get_gripper().command(0.0)
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
        
    def open_gripper(self):
        rospy.loginfo("opening gripper")
        self._robot.get_gripper().command(1.2)
        
    def close_gripper(self):
        self._robot.get_gripper().apply_force(0.7)

        # self._robot.get_gripper().set_distance(0.0)
    
    
    def say(self, text):
        # if self._enable_speech:
        self._output_speech.publish(False, False, 1, text)
        # else:
        #     rospy.loginfo("Say: %s", text)

    def pre_grasp_pose(self):
        # DONT CHANGE THESE VALUES SINCE ALL POINTS is RELATIVE TO HANDPALM LINK
        jointValues = {
            'arm_flex_joint': 0,
            'arm_lift_joint': 0,
            'arm_roll_joint': 0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': -1.57
        }
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
    def openGraspCB(self,data):
        rospy.loginfo("im in cb")
        self.open_gripper()
    
    def gazeCb(self, targetgPointStamped):
        rospy.loginfo(targetgPointStamped)
        p = geometry.Vector3(x = targetgPointStamped.point.x, y = targetgPointStamped.point.y, z = targetgPointStamped.point.z)
        self._robot.get_whole_body().gaze_point(p, targetgPointStamped.header.frame_id)
    def grabBagCB(self, targetPointStamped):
        self._robot.get_whole_body().move_to_neutral()
        initial_wrist_pressure = self.wrist_pressure  

        # Move up the arm to minimise the chance for colliding later on
        jointValues = {
            'wrist_flex_joint': 0,
        }
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
        jointValues = {
                'arm_lift_joint': 0.5,
                }
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
        # wait till it finish last motion
        rospy.loginfo("happen")

        try:
            trans = self.tfBuffer.lookup_transform("base_footprint",targetPointStamped.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(targetPointStamped, trans)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            return
        
        up_pose = geometry.pose(x=transformed_point_stamped.point.x,
                            y=transformed_point_stamped.point.y,
                            z=min(max(float(transformed_point_stamped.point.z+ 0.3),0.02),1.00),
                            ei=-pi,
                            ej=0,
                            ek=-pi/2)

        line_traj = (0, 0, 1)
        # 0.06 to counter effect the over reaching
        line_dis = up_pose.pos.z - transformed_point_stamped.point.z - PALM_SIZE

        try:
            self.open_gripper()
            self._robot.get_whole_body().end_effector_frame = u'hand_palm_link'
            a = self._robot.get_whole_body().move_end_effector_pose(up_pose,
                                                    "base_footprint")
            self._robot.get_whole_body().move_end_effector_by_line(line_traj,line_dis)
            self.close_gripper()

            rospy.loginfo("...done. At the jacket")
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            self.move_to_receive_pose()
            
        self._robot.get_whole_body().move_to_neutral()
                
        if self._robot.get_gripper().get_distance() >0.4 or self.wrist_pressure - initial_wrist_pressure > 0.4:
            rospy.logwarn("successfully picked up")
        else:
            rospy.logwarn("Failed to pick up")
            
        self.move_to_carry_pose()
        self.manip_publisher.publish("Success")
        
    
    
    '''
    Reach the target point and grasp the object
    Reach from the front side of the object, then move the hand towards it to grasp.
    Work for cylindrical object
    '''
    def graspFront(self, targetPointStamped):
        self._robot.get_whole_body().move_to_neutral()
        self.initial_wrist_pressure = self.wrist_pressure        
        rospy.loginfo("happen")
        rospy.loginfo(targetPointStamped)
        try:
            trans = self.tfBuffer.lookup_transform("base_footprint",targetPointStamped.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(targetPointStamped, trans)
            rospy.loginfo(transformed_point_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            return
        
        up_pose = geometry.pose(x=transformed_point_stamped.point.x-0.3,
                            y=transformed_point_stamped.point.y,
                            z=min(max(float(transformed_point_stamped.point.z),0.02),1.00),
                            ei=-pi,
                            ej=-pi/2,
                            ek=0)
        
        line_traj = (0, 0, 1)
        # 0.06 to counter effect the over reaching
        line_dis =  transformed_point_stamped.point.x - up_pose.pos.x - PALM_SIZE
        rospy.loginfo(up_pose)
        try:
            self.open_gripper()
            self._robot.get_whole_body().end_effector_frame = u'hand_palm_link'
            a = self._robot.get_whole_body().move_end_effector_pose(up_pose,
                                                    "base_footprint")
            self._robot.get_whole_body().move_end_effector_by_line(line_traj,line_dis)
            self.close_gripper()
            rospy.loginfo(a)
            rospy.loginfo("...done. At the jacket")
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            self.move_to_receive_pose()
            
        self._robot.get_whole_body().move_to_neutral()
                
        if self._robot.get_gripper().get_distance() >0.4 or self.wrist_pressure - self.initial_wrist_pressure > 0.4:
            rospy.logwarn("successfully picked up")
        else:
            rospy.logwarn("Failed to pick up")
        
        
        self.move_to_carry_pose()
        self.manip_publisher.publish("Success")
    
        """
==================================================================================================================================================================================================================================================================
Services start here
        """
    def wrist_rotate(self, data):
        rospy.loginfo("Got into wrist_rotate function")
        wrist_roll_joint = self._robot.get_whole_body().joint_positions.get('wrist_roll_joint')
        rospy.loginfo(f'{wrist_roll_joint}')
        if wrist_roll_joint > 0:
            wrist_roll_joint = wrist_roll_joint - 1.5708
            rospy.loginfo("big")
        else:
            wrist_roll_joint = wrist_roll_joint + 1.5708
            rospy.loginfo("small")

        jointValues = {
            'wrist_roll_joint': wrist_roll_joint,
        }
        self._robot.get_whole_body().move_to_joint_positions(jointValues)
        rospy.loginfo("wrist_rotate function complete")
        return True
        
        


    '''
    moves the end effector in the plane of the end effector by a certain distance
    It is based on the hand cam image
    The image it reference to(for direction) is the image being rotated by 90 degree counter clockwise
    '''
    def end_effector_plane_move(self, data):
        move_dist = data.distance
        line_traj = (0,0,0)
        if data.direction == "up":
            line_traj = (1,0,0)
        elif data.direction == "down":
            line_traj = (-1,0,0)
        elif data.direction == "left":
            line_traj = (0,-1,0)
        elif data.direction == "right":
            line_traj = (0,1,0)
        else:
            return False
        
        try:
            self._robot.get_whole_body().move_end_effector_by_line(line_traj,move_dist)
            return True
        except Exception as e:
            rospy.logwarn("Failed moving hand in this direction. {}".format(e))
            return False
        pass

    
    '''
    A service to get to the appraoching pose to grasp an object
    data: direction, target_point
    direction: top|front
    target_point: the point to grasp, POINTSTAMPED
    '''
    def approach_object(self, data):
        rospy.loginfo("approach object")
        rospy.loginfo(data)
        direction = data.direction
        target_point = data.target_point
        self._robot.get_whole_body().move_to_neutral()
        # update the wrist pressure value for empty hand. Can be updated to take averages
        self.initial_wrist_pressure = self.wrist_pressure        

        try:
            trans = self.tfBuffer.lookup_transform("base_footprint",target_point.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(target_point, trans)
            rospy.loginfo(transformed_point_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            return False,0
        line_dis =  0
        # calculating the approaching pose based on the directions
        if direction == "top":
            pose = geometry.pose(x=transformed_point_stamped.point.x,
                            y=transformed_point_stamped.point.y,
                            z=min(max(float(transformed_point_stamped.point.z+ 0.2),0.02),1.00),
                            ei=-pi,
                            ej=0,
                            ek=-pi/2)
            line_dis =  pose.pos.z - transformed_point_stamped.point.z - PALM_SIZE
        elif direction == "front":
            pose = geometry.pose(x=transformed_point_stamped.point.x-0.3,
                                y=transformed_point_stamped.point.y,
                                z=min(max(float(transformed_point_stamped.point.z),0.02),1.00),
                                ei=-pi,
                                ej=-pi/2,
                                ek=0)
            line_dis =  transformed_point_stamped.point.x - pose.pos.x
        elif direction == "front-vertical":
            pose = geometry.pose(x=transformed_point_stamped.point.x-0.3,
                                y=transformed_point_stamped.point.y,
                                z=min(max(float(transformed_point_stamped.point.z),0.02),1.00),
                                ei=-pi/2,
                                ej=0,
                                ek=-pi/2)
            line_dis =  transformed_point_stamped.point.x - pose.pos.x
        else:
            return False,0
        

        rospy.loginfo(pose)
        try:
            self.open_gripper()
            # move to that pose
            self._robot.get_whole_body().end_effector_frame = u'hand_palm_link'
            a = self._robot.get_whole_body().move_end_effector_pose(pose,
                                                    "base_footprint")
            # return the value if success, otherwise the error will be captured and false will be returned
            return True, line_dis
            
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            #self.move_to_receive_pose()
            return False,0

    '''
    Move the arm along a line. Using the given distance and close the grip
    data: distance
    '''
    def grasp_object(self,data):
        line_traj = (0, 0, 1)
        line_dis =  data.forward_distance
        try:
            self.open_gripper()
            self._robot.get_whole_body().end_effector_frame = u'hand_palm_link'

            self._robot.get_whole_body().move_end_effector_by_line(line_traj,line_dis)
            self.close_gripper()
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            self.move_to_receive_pose()
            return False
        return True
    
    '''
    retract the arm to the carry pose
    '''
    def retract_arm(self,data):
        try:
            self._robot.get_whole_body().move_to_neutral()
            if self._robot.get_gripper().get_distance() >0.4 or self.wrist_pressure - self.initial_wrist_pressure > 2:
                rospy.logwarn("successfully picked up")
            else:
                rospy.logwarn("Failed to pick up")
                return False
            self.move_to_carry_pose()
        except Exception as e:
            return False
        return True
            

    '''
    Position the end effector to the given point, then release the grip
    data: target_point, POINTSTAMPED
    '''
    def placing_down(self, data):
        rospy.sleep(4)
        target_point = data.target_point
        direction = data.direction
        self._robot.get_whole_body().move_to_neutral()
        
        # update the wrist pressure value for current hand(should hold item). Can be updated to take averages
        holding_wrist_pressure = self.wrist_pressure   
        try:
            trans = self.tfBuffer.lookup_transform("base_footprint",target_point.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(target_point, trans)
            rospy.loginfo(transformed_point_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            return False

        
        if direction == "top":
            pose = geometry.pose(x=transformed_point_stamped.point.x,
                            y=transformed_point_stamped.point.y,
                            z=min(max(float(transformed_point_stamped.point.z+ 0.08),0.02),1.00),
                            ei=-pi,
                            ej=0,
                            ek=-pi/2)
            line_dis = pose.pos.z - transformed_point_stamped.point.z - PALM_SIZE
            line_traj = (0,0,1)
        elif direction == "front":
            pose = geometry.pose(x=transformed_point_stamped.point.x,
                                y=transformed_point_stamped.point.y,
                                z=min(max(float(transformed_point_stamped.point.z+ 0.1),0.02),1.00),
                                ei=-pi,
                                ej=-pi/2,
                                ek=0)
            line_traj = (-1,0,0)
            line_dis = pose.pos.z - transformed_point_stamped.point.z
        else:
            return False
        try:
            # move to that pose
            self._robot.get_whole_body().end_effector_frame = u'hand_palm_link'
            a = self._robot.get_whole_body().move_end_effector_pose(pose,
                                                    "base_footprint")
            self._robot.get_whole_body().move_end_effector_by_line(line_traj,line_dis)
            self.open_gripper()
            # if drop from top do twist
            if direction == "top":
                rospy.loginfo("Got into wrist_rotate function")
                wrist_roll_joint = self._robot.get_whole_body().joint_positions.get('wrist_roll_joint')
                original_joint_Values = jointValues = {
                        'wrist_roll_joint': wrist_roll_joint
                    }
                if wrist_roll_joint > 0:
                    jointValues = {
                        'wrist_roll_joint': wrist_roll_joint - pi/9,
                    }
                else:
                    jointValues = {
                        'wrist_roll_joint': wrist_roll_joint + pi/9,
                    }
                try:
                    self._robot.get_whole_body().move_to_joint_positions(jointValues)
                    self._robot.get_whole_body().move_to_joint_positions(original_joint_Values)
                except:
                    return False 
            rospy.sleep(1)
            # move back to original pose
            self._robot.get_whole_body().move_to_neutral()
            rospy.sleep(1)
            
            # drop success if pressure difference found or does not ohave bag(pressure close to initial pressure that does not holding bag)
            if holding_wrist_pressure - self.wrist_pressure >= 2 or abs(holding_wrist_pressure-self.initial_wrist_pressure) <= 1:
                return True 
            return False
            
        except Exception as e:
            self.open_gripper()
            rospy.logwarn("Failed moving body. {}".format(e))
            return False
    
    '''
    Position the end effector to the given point, then release the grip
    data: target_point, POINTSTAMPED
    A safer way to place down onto the table with 3 line trajectory
    '''
    def placing_down_table(self, data):
        target_point = data.target_point
        direction = data.direction
        buffer = 0.10
        self._robot.get_whole_body().move_to_neutral()
        rospy.sleep(1)

        # update the wrist pressure value for current hand(should hold item). Can be updated to take averages
        try:
            trans = self.tfBuffer.lookup_transform("hand_palm_link",target_point.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(target_point, trans)
            rospy.loginfo(transformed_point_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            return False
        
        try:
            
            self._robot.get_whole_body().end_effector_frame = u'hand_palm_link'
            # move to match height, buffer up a little bit to avoid distance. Makesure the buffer is less than level height if in shelf. 
            self._robot.get_whole_body().move_end_effector_by_line((1,0,0),transformed_point_stamped.point.x+ buffer)

            # move to match horizontally
            self._robot.get_whole_body().move_end_effector_by_line((0,1,0),transformed_point_stamped.point.y)

            # move to match depth
            self._robot.get_whole_body().move_end_effector_by_line((0,0,1),transformed_point_stamped.point.z)

            # move down by buffer
            self._robot.get_whole_body().move_end_effector_by_line((1,0,0), - buffer)
            self.open_gripper()
            
            # move up 
            self._robot.get_whole_body().move_end_effector_by_line((1,0,0), buffer)
            rospy.sleep(0.5)
            self._robot.get_whole_body().move_end_effector_by_line((0,0,-1),transformed_point_stamped.point.z)
            
            rospy.sleep(1)
            # move back to original pose
            self._robot.get_whole_body().move_to_neutral()
            rospy.sleep(1)
            

            return True
            
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            return False
        

    '''
    go to receive pose and wait for a bag to be placed to its hand
    '''
    def receive_pose_for_bag(self, data):
        self.move_to_receive_pose()
        self.open_gripper()
        # in receiving pose, the pressure goes down when pressed
        empty_reading = self.grip_pressure
        t_start = rospy.get_time()
        while not rospy.is_shutdown():
            if empty_reading - self.grip_pressure > 3:
                rospy.loginfo("Bag placed")
                # for safety. Close the grip two seconds after placing
                rospy.sleep(2)
                # move to carry
                self.close_gripper()
                self.move_to_carry_pose()
                return True
            if rospy.get_time() - t_start > 30:
                return False
                
            
    '''
    go to give pose and wait for a bag to be taken from its hand
    '''
    def give_pose_for_bag(self, data):
        self.move_to_receive_pose()
        self.open_gripper()
        # in receiving pose, the pressure goes down when pressed
        hand_full_reading = self.grip_pressure
        t_start = rospy.get_time()
        while not rospy.is_shutdown():
            if self.grip_pressure - hand_full_reading > 3:
                rospy.loginfo("Bag given to person")
                # for safety. Close the grip two seconds after placing
                rospy.sleep(2)
                # move to carry
                self.task_init_pose()
                self.close_gripper()
                return True
            if rospy.get_time() - t_start > 15:
                self.task_init_pose()
                self.close_gripper()
                return False
    def change_to_view_pose(self, data):
        arm_lift_joint_var_MAX =100
        arm_lift_joint_var_MIN =-100
        head_tilt_joint_var_MAX = 100
        head_tilt_joint_var_MIN = -100

        arm_lift_joint_var = min (max(data.arm_lift_joint, arm_lift_joint_var_MIN), arm_lift_joint_var_MAX)
        head_tilt_joint_var = min (max(data.head_tilt_joint, head_tilt_joint_var_MIN), head_tilt_joint_var_MAX)
        try:
            jointValues = {
                "arm_flex_joint": -0.48,
                "arm_lift_joint": arm_lift_joint_var,
                "arm_roll_joint": -1.63,
                "hand_motor_joint": 0.69,
                "head_pan_joint": 1.53,
                "head_tilt_joint": head_tilt_joint_var,
                "wrist_flex_joint": -1.57,
                "wrist_roll_joint": 1.91
            }
            self._robot.get_whole_body().move_to_joint_positions(jointValues)
            return True
        except Exception as e:
            rospy.loginfo(e)
            return False
    def debug(self,data):
        rospy.wait_for_service("unsw/manip/receive_pose")
        a = rospy.ServiceProxy("unsw/manip/receive_pose", receive_pose)
        x = a()
        print(x)

    def test_hand_impedance(self,data):
        self._robot.get_whole_body().move_to_neutral()
        #self._robot.get_gripper().apply_force(0.3)

        up_pose = geometry.pose(x=1,
                            y=0,
                            z=0.1,
                            ei=-pi,
                            ej=-pi/2,
                            ek=0)
        self._robot.get_whole_body().move_end_effector_pose(up_pose,"base_footprint")
        rospy.logwarn(f"2{self._robot.get_whole_body().impedance_config}")
        self._robot.get_whole_body().move_end_effector_by_line((0, 0, 1), 0.2)
if __name__=='__main__':
    rospy.init_node('luggage_grasper_node')  
    luggageGrasper()
     
    rospy.spin()
    # main()        
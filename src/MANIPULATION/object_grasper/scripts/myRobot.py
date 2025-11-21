#!usr/bin/env python3
import hsrb_interface
import rospy
from hsrb_interface import geometry

class MyRobot():
    def __init__(self) -> None:
        self.robot = hsrb_interface.Robot()
        
        self._whole_body = self.robot.get('whole_body')
        
        self._whole_body.linear_weight = 100.0
        self._whole_body.angular_weight = 100.0
        
        
        self._gripper= self.robot.get('gripper')
        
        # set up collision world
        cw = self.robot.get("global_collision_world")
        print(f"next id {cw.next_object_id}")
        # Don't remove the line, it's necessary to wait for the collision world to be ready. Can adjust this to be a busy waiting loop later on
        rospy.sleep(2)
        
        cw.remove_all()
        # attach the box
        print(cw.add_box(x=5, y=5, z=10, pose=geometry.pose(x=0.4, y=0, z=0.2), frame_id='base_link'))
        
        #print(cw.add_box(x=5, y=5, z=10, pose=geometry.pose(x=0.4, y=0, z=0.2), frame_id='base_link'))
        #print(cw.add_sphere(radius=20, pose=geometry.pose(x=20.0, y=1.0, z=0.5),frame_id='base_link'))
    
        #self._whole_body.collision_world = cw
        
        # board = self._whole_body.collision_world.add_box(x=0.74, y=1.18, z=0.74, pose=geometry.pose(x=-5.73, y=-1.95, z=0.74), frame_id='map')
    def get_whole_body(self):
        (print(f"collision world{self._whole_body.collision_world}"))
        return self._whole_body
            
    def get_gripper(self):
        return self._gripper

def mysum(a, b):
    return a + b


#!/usr/bin/python

# Copyright 2014 Keith Clark, Peter Robinson
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys,os
import threading
import random
import math
import pedroclient
import time

IS_VERSION_2 = sys.version_info < (3, 0)
if IS_VERSION_2:
    import Queue
    import Tkinter as tk
    import tkFont
else:
    import queue as Queue
    import tkinter as tk
    from tkinter import font as tkFont


message_queue = Queue.Queue()

USING_SOUND = False
# In order to play the sounds you need pygame installed.
try:
    from pygame import mixer
    USING_SOUND = True
except:
    pass

TWO_TASK = False

NUM_ROBOTS = 2

black = 0, 0, 0
grey = 100,100,100
red = 255, 0, 0
blue = 0, 0, 255

ROBOT1_COLOUR = '#0000AA'
ROBOT2_COLOUR = '#b000b0'
BOTTLE_COLOUR = '#00AA00'
ROBOT_COLOUR = '#FFFF00'
CAMERA_ARC_COLOUR = '#C8C800'
GREY = '#AAAAAA'

FONT_SIZE = 20
FONT_NAME = 'Courier'

TING_FILE = 'tingLoud.wav'
SWOOSH_FILE = 'SwooshLoud.wav'

ROBOT1_COLOUR_TRIPLE = (int(ROBOT1_COLOUR[1:3], 16),
                        int(ROBOT1_COLOUR[3:5], 16),
                        int(ROBOT1_COLOUR[5:7], 16))
ROBOT2_COLOUR_TRIPLE = (int(ROBOT2_COLOUR[1:3], 16),
                        int(ROBOT2_COLOUR[3:5], 16),
                        int(ROBOT2_COLOUR[5:7], 16))

DISPLAY_TEXT = "Total: {0} Last Msg: {1}"

TWO_DISPLAY_TEXT = "Total: {0}"

ONE_DISPLAY_TEXT = "Total: {0}"


CAMERA_ANGLE = math.pi/4   # the camera vision angle
HC_ANGLE = CAMERA_ANGLE/2

RADAR_ANGLES = [0, math.pi/2, 3*math.pi/8, math.pi/4,
                -math.pi/2, -3*math.pi/8, -math.pi/4]
RADAR_NAMES = ['c', 'r3','r2','r1','l3','l2','l1']


WIDTH = 1200           # screen width
HEIGHT = 700          # screen height
TEXT_AREA_HEIGHT = 60

BOTTLE_RADIUS = 5

DROP_RADIUS = 50
DROP_POSITION = WIDTH/2, HEIGHT/2

ROBOT_RADIUS = 20

ROBOT_ID_RADIUS = 8

CLOSE_DIST = 40

NEAR_DIST = 80

MESSAGE_TIME = 2000  # in miliseconds
#MESSAGE_TIME = -1 #uncomment this if message stays (until replaced by next one)
DROP_TIME = 600  # in miliseconds

# square of distance between robot centres
SQ_R2R = (2*ROBOT_RADIUS)**2

# square of dist between robot and bottle centres 
SQ_R2C = (ROBOT_RADIUS + BOTTLE_RADIUS)**2 

# square of dist between robot and drop centres 
SQ_R2D = (ROBOT_RADIUS + DROP_RADIUS)**2 

# how close is near
CLOSE = 40

# square of 'close distance' between robots
C_SQ_R2R = (2*ROBOT_RADIUS + CLOSE)**2

# square of close dist between robot and bottle 
C_SQ_R2C = (ROBOT_RADIUS + BOTTLE_RADIUS + CLOSE)**2 

# square of dist between robot and drop 
C_SQ_R2D = (ROBOT_RADIUS + DROP_RADIUS + CLOSE)**2 

# close distance to wall
C_W = ROBOT_RADIUS + CLOSE

SQ_CW = C_W**2

# number of time steps for veering
VEER_STEPS = 80

TIMER_ID = 100
MESSAGE_TIMER_IDS = [0,101,102]
DROP_TIMER_IDS = [0,103,104]

SPEED_FACTOR = 2
RATE_FACTOR = 2

TIME_STEP = 30

def dist2name(dist):
    if dist == 1000000:
        return 'not_see'
    if dist < CLOSE_DIST:
        return 'close_to'
    elif dist < NEAR_DIST:
        return 'near'
    else:
        return 'see'
 
def get_dir(x, y):
    """Return the direction of the vector (x,y) as
    a result in [0,2*pi]
    """
    if abs(x) < 1.0e-20:
        if y > 0:
            odir = math.pi/2
        else:
            odir = 3*math.pi/2
    else:
        odir = math.atan(abs(float(y)/x))
        if x < 0:
            odir = math.pi - odir
        if y < 0:
            odir = 2*math.pi - odir
    return odir

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

def get_rel_dir(x,y,rdir):
    """Return the direction of the vector (x,y) relative to the direction rdir
    The result is in [-pi, pi]
    """
    return normalize_angle(get_dir(x,y) - rdir)
    
def get_rel_xy(obj1, obj2, rdir):
    """Get the x,y cords of obj2 relative to obj1 (using dir as x axis)"""
    x1 = obj2.x - obj1.x
    y1 = obj2.y - obj1.y
    s = math.sin(rdir)
    c = math.cos(rdir)
    return (c*x1 + s*y1, -s*x1 + c*y1)

def get_closest_to(obj1, obj2, rdir):
    """Get the closest distance from obj1 to obj2 in rdir. obj2 
    return 1000000 if not in overlap in rdir direction
    """
    x,y = get_rel_xy(obj1, obj2, rdir)
    r = obj2.radius
    sdiff = r**2 - y**2
    if (sdiff < 0.1) or (x < 0):
        return 1000000
    return x - math.sqrt(sdiff)

def dist_between(obj1, obj2):
    d1 = math.sqrt((obj1.x - obj2.x)**2 + (obj1.y - obj2.y)**2)
    return d1 - obj1.radius - obj2.radius

def dir2text(dist):
    if dist == 1:
        return "dead_centre"
    elif dist == 2:
        return "centre"
    elif dist == 4:
        return "right"
    else:
        return "left"

def approx_dist(n):
    if n < 2:
        return 0
    return 2**(int(round(math.log(n)/math.log(2))))

class WallPoint:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

corner0 = WallPoint(0,0)
corner1 = WallPoint(WIDTH,0)
corner2 = WallPoint(WIDTH,HEIGHT)
corner3 = WallPoint(0,HEIGHT)

def x_inter(x0, y0, x1, y1):
    if (y0 > 0 and y1 < 0) or (y0 < 0 and y1 > 0):
        d = x0 - y0*(x1 - x0)/(y1 - y0)
        if d > 0:
            return d
    return 1000000

def get_closest_wall(obj1, rdir):
    dist = 100000
    x0, y0 = get_rel_xy(obj1, corner0, rdir)
    x1, y1 = get_rel_xy(obj1, corner1, rdir)
    x2, y2 = get_rel_xy(obj1, corner2, rdir)
    x3, y3 = get_rel_xy(obj1, corner3, rdir)
    return min([x_inter(x0, y0, x1, y1), x_inter(x1, y1, x2, y2), 
                x_inter(x2, y2, x3, y3), x_inter(x3, y3, x0, y0)])
        
    


def see_dir(obj1, obj2,  angle = None):
    """ Direction from obj1 (robot) to obj2    
    0  - not seen
    1  - dead_centre
    2  - centre
    4  - right
    8  - left
    """
    if angle is None:
        angle = get_rel_dir(obj2.x - obj1.x, obj2.y - obj1.y, obj1.dir)
    offset = get_dir_edges(obj1.x, obj1.y, obj2.x, obj2.y, obj2.radius)
    left_angle = normalize_angle(angle - offset)
    right_angle = normalize_angle(angle + offset)
    if right_angle < -HC_ANGLE or left_angle > HC_ANGLE:
        return (0, left_angle, right_angle) 
    if -HC_ANGLE/8 < (left_angle + right_angle) < HC_ANGLE/8:
        return (1, left_angle, right_angle)  
    if -HC_ANGLE/2 < (left_angle + right_angle) < HC_ANGLE/2:
        return (2, left_angle, right_angle)  
    elif  (left_angle + right_angle) <=  -HC_ANGLE/2 and \
            right_angle >  -HC_ANGLE:
        return (8, left_angle, right_angle) 
    elif (left_angle + right_angle) >=   HC_ANGLE/2 and \
            left_angle < HC_ANGLE:
        return (4, left_angle, right_angle) 
    else:
        return (0, left_angle, right_angle) 

def get_dir_edges(x1, y1, x2, y2, radius):
    """Return the direction in [0,pi] of the right edge
    of a circular object of given radius whose centre is at
    (x2, y2) from the point (x1,y1)
    """
    hyp = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    if radius > hyp: return HC_ANGLE
    return math.asin(radius/hyp)

def transform(shiftx, shifty, theta, pts):
    """Used for shift and rotation of the polygons representing the 
    robot wheels"""
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    for i in range(len(pts)):
        x,y = pts[i]
        newx = shiftx + x*cos_theta - y*sin_theta
        newy = shifty + x*sin_theta + y*cos_theta
        pts[i] = (newx,newy)



def action_in(a, actions):
    """return true iff actions contain an action with the same functor as a"""
    assert (a.get_type() == pedroclient.PObject.structtype), \
        str(a) + " is not a structure"
    a_val = a.functor.val
    for a1 in actions:
        assert (a1.get_type() == pedroclient.PObject.structtype), \
            str(a1) + " is not a structure"
        if a_val == a1.functor.val:
            return True
    return False

def get_start_stop_actions(old_actions, new_actions):    
    return ([a for a in new_actions if not a in old_actions],
            [a for a in old_actions if not action_in(a, new_actions)])


class GetNotifications(threading.Thread):
    def __init__( self, client, master):
        self.client = client
        self.master = master
        self.running = True
        threading.Thread.__init__(self)

    def run(self):
        """Run as thread - post an event when notifications arrive"""
    
        while (self.running):
            try:
                p2pmsg = self.client.get_term()[0]
                message_queue.put(p2pmsg)
                self.master.event_generate("<<pedro>>")
            except Exception as ex:
                print("Exception ", str(ex))
                break

    def stop(self):
        self.running = False
    

class Drop(object):
    """A drop class so teh drop bottle be treated like a bottle object."""
    def __init__(self):
        self.x, self.y = DROP_POSITION
        self.radius = DROP_RADIUS

class Robot(object):
    def __init__(self, env, ID, x, y):
        self.x,self.y = x, y
        self.dir = 0
        self.master = env.parent._master
        self.holding = False
        self.last_holding = True
        self.gripper_open = True
        self.last_gripper_open = False
        self.moving = False
        self.percept_update = False
        self.turning = 0
        self.touching = 0
        self.saved_touching = 0
        self.touching_wall = 0
        self.saved_touching_wall = 0
        self.at_drop = False
        self.close_drop = False
        self.close_bottle = False
        self.close_robot = False
        self.speed = 1.5
        self.veer_steps = 0
        self.veering = False
        self.env = env
        self.ID = ID
        self.drop = Drop()
        self.over = False
        self.queued_messages = []
        self.radius = ROBOT_RADIUS
        self.saved_robot_see_dist = (1000000, 0)
        self.saved_robot_see_dist = (1000000, 0)
        self.saved_drop_see_dist = (1000000, 0)
        self.saved_bottle_see_dist = (1000000, 0)
        self.saved_radar_right = -1
        self.saved_radar_left = -1
        self.saved_radar_centre = -1
        self.stopped = False
        self.message = ''
        self.dropped_can = None
        self.started = False
        self.count = 0

    def delete_message(self):
        self.message = ''

    def delete_drop(self):
        if self.dropped_can is not None:
            if USING_SOUND:
                self.env.parent.swoosh_sound.play()
            self.env.removeBottleID(self.dropped_can)
            self.dropped_can = None

    def add_holding_message(self, rf):
        if TWO_TASK:
            msg = "{0}(holding(r{1}))".format(rf, self.ID)
        else:
            msg = "{0}(holding())".format(rf)
        self.queued_messages.append(msg)

    def add_gripper_message(self, rf):
        if TWO_TASK:
            msg = "{0}(gripper_open(r{1}))".format(rf, self.ID)
        else:
            msg = "{0}(gripper_open())".format(rf)
        self.queued_messages.append(msg)

    def add_drop_message(self, rf):
        if TWO_TASK and rf == 'u_':
            msg = "{0}(over_drop(!(r{1})))".format(rf, self.ID)
        elif TWO_TASK:
            msg = "{0}(over_drop(r{1}))".format(rf, self.ID)
        else:
            msg = "{0}(over_drop())".format(rf)
        self.queued_messages.append(msg)

    def add_see_message(self, rf, thing, dist, dir_):
        if TWO_TASK and rf == 'u_':
            msg = "{0}(see_at(!(r{1}), {2}, {3}, {4}))".format(rf, self.ID,
                                                            thing, dist,
                                                            dir_)
        elif TWO_TASK:
            msg = "{0}(see_at(r{1}, {2}, {3}, {4}))".format(rf, self.ID,
                                                            thing, dist,
                                                            dir_)            
        else:
            msg = "{0}(see_at({1}, {2}, {3}))".format(rf, thing, dist, dir_)
        self.queued_messages.append(msg)

    def add_message(self, msg):
        if TWO_TASK:
            self.queued_messages.append('msg(robot{0}, {1})'.format(self.ID, 
                                                                    msg))
        else:
            self.queued_messages.append(msg)

    def flipStopped(self):
        self.stopped = not self.stopped

    def send_msg(self, msg):
        self.env.send_msg(self.ID, msg)
        
    def send_messages(self):
        if self.queued_messages:
            message = '['
            for m in self.queued_messages:
                message += m + ','
            message = message[:-1]+']'
            self.send_msg(message)
            self.queued_messages = []
 
    def check_seeing_drop(self, see_robots):
        """Check if seeing drop - send off percept.
        see_robots is a list of  (ID, left, right) where left and right are the
        relative angles to the robot"""
        see = 0
        # angle to the centre of the drop
        angle = get_rel_dir(self.drop.x - self.x, self.drop.y - self.y, 
                            self.dir)
        see, left, right = see_dir(self, self.drop, angle)
        if -math.pi/2 < angle < math.pi/2 and \
                not self.hidden(see_robots, self.drop, angle, left, right):
            if see > 0:
                d = dist_between(self, self.drop)
                self.see_dist.append((d, see, 'drop'))
        
    def check_seeing(self):
        """Check to see what is seen"""
        see_robots = []
        self.see_dist = []
        # check to see if a robot bottle be seen and 
        # construct the see_robots list to test for hiding
        for robot in self.env.robots:
            if robot == self:
                continue
            see, left, right = see_dir(self, robot)
            if left > -math.pi/2 and right < math.pi/2:
                if see > 0:
                    d = dist_between(self, robot)
                    self.see_dist.append((d, see, 'robot'))
                    see_robots.append((robot, left, right))

        self.check_seeing_drop(see_robots)

        # check if seeing a bottle
        for bottle in self.env.bottles:
            angle = get_rel_dir(bottle.x - self.x, bottle.y - self.y, self.dir)
            see, left, right = see_dir(self, bottle, angle)
            if -math.pi/2 < angle < math.pi/2 and \
                    not self.hidden(see_robots, bottle, angle, left, right):
                if see > 0:
                    d = dist_between(self, bottle)
                    self.see_dist.append((d, see, 'bottle'))
        self.see_dist.sort()
        robot_dir = 0
        robot_dist = 1000000
        drop_dir = 0
        drop_dist = 1000000
        bottle_dir = 0
        bottle_dist = 1000000
        for see_dist, sdir, name in self.see_dist:
            if name == 'robot' and robot_dist == 1000000:
                robot_dist = approx_dist(see_dist)
                robot_dir = dir2text(sdir)
            if name == 'drop' and drop_dist == 1000000:
                drop_dist = approx_dist(see_dist)
                drop_dir = dir2text(sdir)
            if name == 'bottle' and bottle_dist == 1000000:
                bottle_dist = approx_dist(see_dist)
                bottle_dir = dir2text(sdir)
   

        robot_info = (robot_dist, robot_dir)
        if self.saved_robot_see_dist != robot_info:
            if self.saved_robot_see_dist[0] != 1000000 and\
               robot_dist == 1000000:
                self.add_see_message('f_', 'robot',
                                      self.saved_robot_see_dist[0], 
                                      self.saved_robot_see_dist[1])
            else:
                self.add_see_message('u_', '!(robot)', robot_dist, robot_dir)
            self.saved_robot_see_dist = robot_info
        drop_info = (drop_dist, drop_dir)
        if self.saved_drop_see_dist != drop_info:
            if self.saved_drop_see_dist[0] != 1000000 and\
               drop_dist == 1000000:
                self.add_see_message('f_', 'drop',
                                      self.saved_drop_see_dist[0], 
                                      self.saved_drop_see_dist[1])
            if drop_dist !=1000000:
                self.add_see_message('u_', '!(drop)', drop_dist, drop_dir)
            self.saved_drop_see_dist = drop_info
        bottle_info = (bottle_dist, bottle_dir)
        if self.saved_bottle_see_dist != bottle_info:
            if self.saved_bottle_see_dist[0] != 1000000 and\
               bottle_dist == 1000000:
                self.add_see_message('f_', 'bottle',
                                      self.saved_bottle_see_dist[0], 
                                      self.saved_bottle_see_dist[1])
            if bottle_dist !=1000000:
                self.add_see_message('u_', '!(bottle)', bottle_dist, bottle_dir)
            self.saved_bottle_see_dist = bottle_info

    def hidden(self, see_robots, obj, obj_angle, left, right):
        """Test to see if obj and obj_angle is hidden by a robot"""
        dist2obj = math.sqrt((self.x - obj.x)**2 + (self.y - obj.y)**2)
        rel_left = normalize_angle(left - obj_angle)
        rel_right = normalize_angle(right - obj_angle)
        for robot, rleft, rright in see_robots:
            dist2robot = math.sqrt((self.x - robot.x)**2 + (self.y - robot.y)**2)
            if dist2robot < dist2obj:
                rel_rleft = normalize_angle(rleft - obj_angle)
                rel_rright = normalize_angle(rright - obj_angle)
                if -math.pi/2 < rel_rleft < rel_left and \
                        math.pi/2 > rel_rright > rel_right:
                    return True
        return False

    
    # for touching
    # self.touching = 0    - not touching
    # OW  or together
    #   1  - touching centre
    #   2  - touching right
    #   4  - touching left
    #   8  - touching far_right
    #   16  - touching far_left

    def check_touching(self):
        """Determine touching and send as msg"""
        self.touching = 0
        self.touching_wall = 0
        self.check_touching_bottle()
        self.check_touching_robot()
        self.check_touching_walls()
        if self.touching != self.saved_touching:
            diff = self.saved_touching & ~self.touching
            # to forget
            if diff & 1 == 1:
                self.add_message('f_(touching(centre))')
            if diff & 2 == 2:
                self.add_message('f_(touching(right))')
            if diff & 4 == 4:
                self.add_message('f_(touching(left))')
            if diff & 8 == 8:
                self.add_message('f_(touching(far_right))')
            if diff & 16 == 16:
                self.add_message('f_(touching(far_left))')
            # to remember
            if self.touching & 1 == 1:
                self.add_message('r_(touching(centre))')
            if self.touching & 2 == 2:
                self.add_message('r_(touching(right))')
            if self.touching & 4 == 4:
                self.add_message('r_(touching(left))')
            if self.touching & 8 == 8:
                self.add_message('r_(touching(far_right))')
            if self.touching & 16 == 16:
                self.add_message('r_(touching(far_left))')
            self.saved_touching = self.touching
        if self.touching_wall != self.saved_touching_wall:
            diff = self.saved_touching_wall & ~self.touching_wall
            # to forget
            if diff & 1 == 1:
                self.add_message('f_(touching_wall(centre))')
            if diff & 2 == 2:
                self.add_message('f_(touching_wall(right))')
            if diff & 4 == 4:
                self.add_message('f_(touching_wall(left))')
            if diff & 8 == 8:
                self.add_message('f_(touching_wall(far_right))')
            if diff & 16 == 16:
                self.add_message('f_(touching_wall(far_left))')
            # to remember
            if self.touching_wall & 1 == 1:
                self.add_message('r_(touching_wall(centre))')
            if self.touching_wall & 2 == 2:
                self.add_message('r_(touching_wall(right))')
            if self.touching_wall & 4 == 4:
                self.add_message('r_(touching_wall(left))')
            if self.touching_wall & 8 == 8:
                self.add_message('r_(touching_wall(far_right))')
            if self.touching_wall & 16 == 16:
                self.add_message('r_(touching_wall(far_left))')
            self.saved_touching_wall = self.touching_wall


    def check_holding(self):
        if self.holding:
            if not self.last_holding:
                self.add_holding_message("r_")
                self.last_holding = self.holding
        else:
            if self.last_holding:
                self.add_holding_message("f_")
                self.last_holding = self.holding

    def check_gripper_open(self):
        if self.gripper_open:
            if not self.last_gripper_open:
                self.add_gripper_message("r_")
                self.last_gripper_open = self.gripper_open
        else:
            if self.last_gripper_open:
                self.add_gripper_message("f_")
                self.last_gripper_open = self.gripper_open

    def check_touching_bottle(self):
        """Check if touching a bottle and if touching in front return index
        of touching bottle or return -1"""
        index = -1
        for i, bottle in enumerate(self.env.bottles):
            angle = get_rel_dir(bottle.x - self.x, bottle.y - self.y, self.dir)
            sdir, left, right = see_dir(self, bottle, angle)
            d = dist_between(self, bottle)
            bottle_dist = approx_dist(d)
            bottle_dir = dir2text(sdir)
            if bottle_dist == 0 and bottle_dir in ['centre', 'dead_centre']:
                index = i
        return index
    
    def update_touching(self, angle):
        """Update touching given the touching angle"""
        if -HC_ANGLE/4 < angle < HC_ANGLE/4:
            self.touching |= 1
        elif HC_ANGLE/4 <= angle < HC_ANGLE:
            self.touching |= 2
        elif -HC_ANGLE/4 >= angle > -HC_ANGLE:
            self.touching |= 4
        elif HC_ANGLE <= angle < math.pi/2:
            self.touching |= 8
        elif -HC_ANGLE >= angle > -math.pi/2:
            self.touching |= 16

    def update_touching_wall(self, angle):
        """Update touching given the touching angle"""
        if -HC_ANGLE/4 < angle < HC_ANGLE/4:
            self.touching_wall |= 1
        elif HC_ANGLE/4 <= angle < HC_ANGLE:
            self.touching_wall |= 2
        elif -HC_ANGLE/4 >= angle > -HC_ANGLE:
            self.touching_wall |= 4
        elif HC_ANGLE <= angle < math.pi/2:
            self.touching_wall |= 8
        elif -HC_ANGLE >= angle > -math.pi/2:
            self.touching_wall |= 16

    def check_touching_robot(self):
        """Update touching for robots"""
        for robot in self.env.robots:
            if robot == self:
                continue
            if (self.x - robot.x)**2 + (self.y -robot.y)**2 < SQ_R2R + 100:
                angle = get_rel_dir(robot.x - self.x, 
                                    robot.y - self.y, self.dir)
                self.update_touching(angle)

    def check_touching_drop(self):
        """Update touching for drop."""
        if (self.x - self.drop.x)**2 + (self.y - self.drop.y)**2 < SQ_R2D + 50:
            angle = get_rel_dir(self.drop.x - self.x, 
                                self.drop.y - self.y, self.dir)
            self.update_touching(angle)


    def check_touching_walls(self):
        """Update touching for walls"""
        if self.x < ROBOT_RADIUS+2:
            # touching West wall
            self.update_touching_wall(get_rel_dir(-1,0,self.dir))
        if self.x > WIDTH - ROBOT_RADIUS - 2:
            # touching E
             self.update_touching_wall(get_rel_dir(1,0,self.dir))
        if self.y <  ROBOT_RADIUS+2:
            # touching N
            self.update_touching_wall(get_rel_dir(0,-1,self.dir))
        if self.y > HEIGHT - ROBOT_RADIUS - 2:
            # touching S
             self.update_touching_wall(get_rel_dir(0,1,self.dir))



    def check_radar(self):
        """Determine radar and send as msg"""
        self.radars = [[],[],[],[],[],[],[]]
        self.check_radar_drop()
        self.check_radar_robot()
        self.check_radar_walls()
        self.check_radar_bottles()
        r1 = self.radars[1]
        r2 = self.radars[2]
        r3 = self.radars[3]
        r1.sort()
        r2.sort()
        r3.sort()
        d = min(r1[0], r2[0], r3[0])
        closest = approx_dist(d)
        if closest < 33 and self.saved_radar_right != closest:
            self.add_message('radar_close(right, %d)' % closest)
            self.saved_radar_right = closest
        elif closest > 33 and self.saved_radar_right != -1:
            self.add_message('radar_not_close(right)')
            self.saved_radar_right = -1
        l1 = self.radars[4]
        l2 = self.radars[5]
        l3 = self.radars[6]
        l1.sort()
        l2.sort()
        l3.sort()
        d = min(l1[0], l2[0], l3[0])
        closest = approx_dist(d)
        if closest < 17 and self.saved_radar_left != closest:
            self.add_message('radar_close(left, %d)' % closest)
            self.saved_radar_left = closest
        elif closest > 17 and self.saved_radar_left != -1:
            self.add_message('radar_not_close(left)')
            self.saved_radar_left = -1
        dists = self.radars[0]
        dists.sort()
        closest = approx_dist(dists[0])
        if self.saved_radar_centre != closest:
            self.add_message('radar_centre_dist(%d)' % closest)
            self.saved_radar_centre = closest

    def check_radar_bottles(self):
        """Check if radars see a bottle"""
        for bottle in self.env.bottles:
            dist = get_closest_to(self, bottle, self.dir)
            self.radars[0].append(dist-ROBOT_RADIUS)
    
    def check_radar_robot(self):
        """Update radar for robots"""
        for robot in self.env.robots:
            if robot == self:
                continue
            self.update_radar(robot)

    def update_radar(self, obj):
        for i,ang in enumerate(RADAR_ANGLES):
            dist = get_closest_to(self, obj, self.dir + ang)
            self.radars[i].append(dist-ROBOT_RADIUS)
  
    def check_radar_drop(self):
        """Update radar for drop."""
        drop = self.drop
        self.update_radar(self.drop)

    def check_radar_walls(self):
        """Update radar for walls"""
        for i,ang in enumerate(RADAR_ANGLES):        
            dist = get_closest_wall(self, self.dir + ang)
            self.radars[i].append(dist-ROBOT_RADIUS)


    def check_over_drop(self):
        x,y = self.drop.x, self.drop.y
        angle = get_rel_dir(x - self.x, 
                            y - self.y, self.dir)
        if -HC_ANGLE < angle < HC_ANGLE and \
           (self.x - x)**2 + (self.y - y)**2 < SQ_R2D:
            if not self.over:
                self.add_drop_message("r_")
                self.over = True
        else:
            if self.over:
                self.add_drop_message("f_")
                self.over = False


    def check_close_drop(self):
        angle = get_rel_dir(self.drop.x - self.x, 
                            self.drop.y - self.y, self.dir)
        close = -HC_ANGLE < angle < HC_ANGLE and \
            (self.x - self.drop.x)**2 + (self.y - self.drop.y)**2 < C_SQ_R2D
        if close:
            if not self.close_drop:
                self.add_message('r_(close(drop))')
                self.close_drop = True
        else:
            if self.close_drop:
                self.add_message('f_(close(drop))')
                self.close_drop = False
 

    def check_close_robot(self):
        """Check if close to a robot"""
        close = False
        for robot in self.env.robots:
            if robot == self:
                continue
            angle = get_rel_dir(robot.x - self.x, robot.y - self.y, self.dir)
            if  -HC_ANGLE < angle < HC_ANGLE \
                    and (self.x - robot.x)**2 + (self.y -robot.y)**2 < C_SQ_R2R:
                # in front and close
                close = True
                break
        if close:
            if not self.close_robot:
                self.add_message('r_(close(robot))')
                self.close_robot = True
        else:
            if self.close_robot:
                self.add_message('f_(close(robot))')
                self.close_robot = False
            
    def check_close_bottle(self):
        """Check if code to bottle - must be in view angle."""
        index = -1
        for i in range(len(self.env.bottles)):
            bottle = self.env.bottles[i]
            angle = get_rel_dir(bottle.x - self.x, bottle.y - self.y, self.dir)
            if -HC_ANGLE < angle < HC_ANGLE \
                   and (self.x - bottle.x)**2 + (self.y -bottle.y)**2 < C_SQ_R2C:
                    index = i
                    break

        if index == -1:
            if self.close_bottle:
                self.add_message('f_(close(bottle))')
                self.close_bottle = False
        else:
            if not self.close_bottle:
                self.add_message('r_(close(bottle))')
                self.close_bottle = True

            
    def real_dir(self):
        if self.speed < 0:
            return -self.dir
        else:
            return self.dir

    def update_speed(self):
        """Update the robot velocity based on touching walls, drop or other robot"""
        # modify angle if turning - keep dir in [0,2*pi]
        if self.turning != 0:
            self.dir += self.turning*0.04
            if self.dir < 0: self.dir += 2*math.pi
            if self.dir > 2*math.pi: self.dir -= 2*math.pi
        # only bother if robot is moving
        if self.moving or self.percept_update:
            self.percept_update = False
            # self the x and y velocity - modified by collisions
            self.vx = self.speed*math.cos(self.dir)
            self.vy = self.speed*math.sin(self.dir)
            # Am I colliding with walls?
            touching_wall = False
            if self.x < ROBOT_RADIUS+2 and self.vx < 0: 
                touching_wall = True
                self.vx = 0
            if self.x > WIDTH - ROBOT_RADIUS-2 and self.vx > 0: 
                touching_wall = True
                self.vx = 0
            if self.y < ROBOT_RADIUS+2 and self.vy < 0:
                touching_wall = True
                self.vy = 0
            if self.y > HEIGHT - ROBOT_RADIUS-2 and self.vy > 0:
                touching_wall = True
                self.vy = 0
            # Am I colliding with the drop
            if self.vx == 0 and self.vy == 0:
                return
            robot_dir = get_dir(self.vx, self.vy)
            # Am I colliding with another robot
            if abs(self.vx) + abs(self.vy) < 0.00001:  # used to be two .vx
                self.vx, self.vy = 0,0
                return
            for robot in self.env.robots:
                if robot == self:
                    continue
                robot_dir = get_rel_dir(robot.x - self.x, robot.y - self.y, self.dir)
                if (robot.x - self.x)**2 + (robot.y - self.y)**2 < SQ_R2R + 8 and -math.pi/2 < robot_dir < math.pi/2:
                    robot_dir = get_dir(robot.x - self.x, robot.y - self.y)
                    speed = -self.vx*math.sin(robot_dir) + \
                        self.vy*math.cos(robot_dir)
                    self.vx = -speed*math.sin(robot_dir)
                    self.vy = speed*math.cos(robot_dir)
                    if touching_wall:
                        self.vx, self.vy = 0, 0
                    break
        else:
            self.vx, self.vy = 0, 0
        

    def update(self):
        """Update the robot position and possibly move bottles"""
        # only bother if robot is moving
        if self.moving:
            if self.veering:
                self.veer_steps -= 1
                if self.veer_steps <= 0:
                    self.veering = False
                    self.stop()
                    self.add_message('end_veering')
                elif self.veer_steps == VEER_STEPS/2:
                    self.turning *= -1
            # check bottles to see if they need to move out of the way
            dir = get_dir(self.vx, self.vy)
            for i in range(len(self.env.bottles)):
                bottle = self.env.bottles[i]
                cx = bottle.x
                cy = bottle.y
                if (self.x - cx)**2 + (self.y -cy)**2 < SQ_R2C + 4:
                    # robot is touching bottle
                    angle = get_dir(cx - self.x, cy - self.y)
                    diff = get_rel_dir(cx - self.x, cy - self.y, dir)
                    if abs(diff) < math.pi/2 or abs(diff) > 3*math.pi/2:
                        # the bottle is in the front half
                        # TODO - robot pushes bottle into something
                        if abs(diff) < 0.2:
                            # the bottle is straight in front so move with robot
                            dx = self.speed*math.cos(dir)
                            dy = self.speed*math.sin(dir)
                            self.env.bottles[i].move_delta(dx, dy)
                        else:
                            # bottle moves around robot (tangent)
                            cspeed = self.speed*math.cos(diff)
                            self.env.bottles[i].move_delta(cspeed*math.cos(angle),
                                cspeed*math.sin(angle))
            
            self.x += self.vx
            self.y += self.vy
        # now that things have changed recheck
        self.check_seeing()
        self.check_touching()
        self.check_over_drop()
        self.check_holding()
        self.check_gripper_open()
        self.send_messages()


    def move_forward(self, speed):
        self.speed = speed*SPEED_FACTOR
        self.moving = True
        
    def turn(self, tdir, rate):
        if tdir == 'left' or tdir == 'far_left':
            self.turning = -rate*RATE_FACTOR
        else:
            self.turning = rate*RATE_FACTOR


    def start_veering(self, dir):
        self.turn(dir, 0.1)
        self.move_forward(1.2)
        self.veer_steps = VEER_STEPS
        self.veering = True

    def stop(self):
        self.moving = False
        self.turning = 0
        if  self.veering:
            self.add_message('end_veering')
            self.veering = False
        
    def stop_turn(self):
        self.turning = 0

    def stop_move(self):
        self.moving = False
        if  self.veering:
            self.add_message('end_veering')
            self.veering = False
        
    def grip(self):
        self.gripper_open = False
        index = self.check_touching_bottle()
        if index == -1 : return
        self.holding = True
        self.env.bottles.pop(index)
        self.percept_update = True


    def release(self):
        self.gripper_open = True
        if self.holding: 
            self.holding = False
            if USING_SOUND:
                self.env.parent.ting_sound.play()
            d = self.dir
            x = self.x + math.cos(d)*(ROBOT_RADIUS + BOTTLE_RADIUS)
            y = self.y + math.sin(d)*(ROBOT_RADIUS + BOTTLE_RADIUS)
            self.dropped_can = self.env.addBottle(x,y)
            self.master.after(DROP_TIME,self.delete_drop)

            
    def unset_holding(self):
        self.holding = False
        
class Bottle(object):
    def __init__(self, env, x, y):
        self.x = x
        self.y = y
        self.ID = env.inc_ID()
        self.radius = BOTTLE_RADIUS

    def move_delta(self, dx, dy):
        self.x += dx
        self.y += dy

    def move(self, x, y):
        self.x = x
        self.y = y

class Environment(object):
    def __init__(self, parent):
        self.parent = parent

        self.bottles = []
        self.bottleID = 0
        self.leftdown= False
        self.messages_to_send = None
        robot0 = Robot(self, 1, WIDTH*0.25, HEIGHT/2)
        if NUM_ROBOTS == 1:
            self.robots = [robot0]
        else:
            robot1 = Robot(self, 2, WIDTH*0.75, HEIGHT/2)
            self.robots = [robot0, robot1]
        self.task_actions = {}

    def inc_ID(self):
        self.bottleID += 1
        return self.bottleID

    def flipStopped(self):
        for r in self.robots:
            r.flipStopped()
            
    def bottle_place(self, x, y):
        dx,dy = DROP_POSITION
        if x > dx - DROP_RADIUS and y <  dy + DROP_RADIUS:
            return False
        return ROBOT_RADIUS < x < WIDTH - ROBOT_RADIUS and \
            ROBOT_RADIUS < y < HEIGHT - ROBOT_RADIUS

    def addBottle(self, x, y):
        bottle = Bottle(self, x,y)
        self.bottles.append(bottle)
        return bottle.ID

    def getBottle(self, x, y):
        for i in range(len(self.bottles)):
            bottle = self.bottles[i]
            if abs(bottle.x - x) < BOTTLE_RADIUS and abs(bottle.y - y) < BOTTLE_RADIUS:
                return bottle
        return None

    def removeBottleID(self, BID):
        for b in self.bottles:
            if b.ID == BID:
                self.bottles.remove(b)
                for robot in self.robots:
                    robot.check_seeing()
                    robot.check_touching()
                break

    def removeBottle(self, x, y):
        for i in range(len(self.bottles)):
            bottle = self.bottles[i]
            if abs(bottle.x - x) < BOTTLE_RADIUS and abs(bottle.y - y) < BOTTLE_RADIUS:
                self.bottles.pop(i)
                for robot in self.robots:
                    robot.check_seeing()
                    robot.check_touching()
                    #robot.check_radar()
                break
        # for robot in self.robots:
        #     if robot.holding and  \
        #             abs(robot.x - x) < ROBOT_RADIUS and \
        #             abs(robot.y - y) < ROBOT_RADIUS:
        #         robot.holding = False


    def step(self):
        for r in self.robots:
            if r.started:
                r.update_speed()
                r.update()

 

    def process_msg(self, term):
        msg = term.args[2]
        # actions(sender_task, actions) if robotic action message
        #print("_____msg____",str(term))
        if TWO_TASK:
            sender_process = term.args[1].args[0].args[1].val
            self.sender_process = sender_process
            robotID = 5
        elif NUM_ROBOTS == 1:
            sender_process = term.args[1].args[0].args[1].val
            self.sender_process = sender_process
            robotID = 0
        else:
            sender_process = term.args[1].args[0].args[1].val
            self.sender_process = sender_process[:-1]
            robotID = int(sender_process[-1])-1
        #print(robotID+1, "msg", str(msg))
        if str(msg) == "initialise_":
            if TWO_TASK:
                self.robots[0].started = True
                self.robots[1].started = True
            elif NUM_ROBOTS == 1:
                self.robots[0].started = True
            else:
                 self.robots[robotID].started = True
            return
        elif  msg.get_type() == pedroclient.PObject.atomtype and \
                msg.val == 'finish':
            return
        elif  msg.get_type() == pedroclient.PObject.structtype and \
                msg.functor.val ==  'display_info':
            cmd = msg.args[0].functor.val
            if not TWO_TASK:
                self.robots[robotID].message = str(msg.args[0])
                if (MESSAGE_TIME > 0):
                    self.parent._master.after(MESSAGE_TIME, 
                                              self.robots[robotID].delete_message)
            if cmd == 'count':
                cmd_args = msg.args[0].args
                print("count {}".format(cmd_args[0].val))
                if TWO_TASK or (NUM_ROBOTS == 1):
                    self.robots[0].count = cmd_args[0].val
                else:
                    self.robots[robotID].count = cmd_args[0].val
 

        elif msg.get_type() == pedroclient.PObject.structtype and \
             msg.functor.val == 'actions':
            action_term = msg.args[1]
            if TWO_TASK:
                robotID = int(msg.args[0].val[-1]) - 1
            if action_term.get_type() == pedroclient.PObject.listtype:
                actions = action_term.toList()
            else:
                actions = []
            old_actions = self.task_actions.get(robotID, [])
            self.task_actions[robotID] = actions
            (start_actions, stop_actions) \
                = get_start_stop_actions(old_actions, actions)
            self.process_stop_actions(stop_actions, robotID)
            self.process_start_actions(start_actions, robotID)
            return
        else:
            assert (False), "COMMAND: {}".format(str(msg))
        

    def process_stop_actions(self, stop_actions, robotID):
        for a in stop_actions:
            print("r{0} : stop_action".format(robotID), str(a))
            action_args = a.args
            cmd = a.functor.val
            if cmd in ['close_gripper', 'open_gripper']:
                pass
            elif cmd == 'turn':
                self.robots[robotID].stop_turn()
            elif cmd == 'move':
                self.robots[robotID].stop_move()
            else:
                assert (False), "COMMAND: {}".format(str(a))

    def process_start_actions(self, start_actions, robotID):
        for a in start_actions:
            print("r{0} : start_action".format(robotID), str(a))
            cmd_args = a.args
            cmd = a.functor.val
            if cmd == 'close_gripper':
                self.robots[robotID].grip()
            elif cmd == 'open_gripper':
                self.robots[robotID].release()
            elif cmd == 'turn':
                self.robots[robotID].turn(cmd_args[0].val, cmd_args[1].val)
            elif cmd == 'move':
                self.robots[robotID].move_forward(cmd_args[0].val)
            else:
                assert (False), "COMMAND: {}".format(str(a))
         
    def send_msg(self, ID, msg):
        if TWO_TASK:
            self.parent.client.p2p("percepts:{0}".format(self.sender_process), msg)
        elif NUM_ROBOTS == 1:
            self.parent.client.p2p("percepts:{0}".format(self.sender_process), msg)
        else:
            self.parent.client.p2p("percepts:{0}{1}".format(self.sender_process, ID), msg)

    def send_message(self, ID, msg):
        if TWO_TASK:
             self.parent.client.p2p("messages:{0}".format(self.sender_process),  msg)
        elif NUM_ROBOTS == 1:
            self.parent.client.p2p("messages:{0}".format(self.sender_process),  msg)
        else:
            self.parent.client.p2p("messages:{0}{1}".format(self.sender_process, ID), msg)

    def send_log(self, msg):
        self.parent.client.p2p("thread0:logger2", '['+msg+',nl]')

class BottlesApp(object):

    def __init__(self, master):
        self._master = master
        master.title("Bottles")
        master.protocol("WM_DELETE_WINDOW", self.onClose)
        master.geometry("{0}x{1}".format(WIDTH, HEIGHT+TEXT_AREA_HEIGHT))
        self.env = Environment(self)
        self.canvas = tk.Canvas(master, bg = 'grey92')
        self.canvas.pack(fill=tk.BOTH, expand=1)
        self.canvas.bind("<Button-1>", self.button_press_cb)
        self.canvas.bind("<Control-Button-1>", self.button_press_add_cb)
        self.canvas.bind("<Control-Shift-Button-1>", self.button_press_rem_cb)
        self.canvas.bind("<ButtonRelease-1>", self.button_release_cb)
        self.canvas.bind("<Button-2>", self.mid_button_press_cb)
        self.canvas.bind("<Button-3>", self.right_button_press_cb)
        self.canvas.bind("<B1-Motion>", self.motion_cb)
        self.canvas.bind_all("<Key>", self.onKeyPress)
        self._master.bind("<<pedro>>", self.data_cb)

        self.font = tkFont.Font(family=FONT_NAME,size=FONT_SIZE,weight="bold")
        self.running = True
        self.client = pedroclient.PedroClient()
        self.client.register('env')
        self.thread = GetNotifications(self.client, master)
        self.thread.setDaemon(True)
        self.thread.start()
        self.moving_bottle = None
        if USING_SOUND:
            mixer.init()
            self.ting_sound = mixer.Sound(TING_FILE)
            self.swoosh_sound = mixer.Sound(SWOOSH_FILE)
        
        self.show()
        self._master.after(TIME_STEP, self.timer_cb)
        
    def onClose(self):
        if TWO_TASK:
            self.env.send_message(1, "terminate_agent")
        else:
            self.env.send_message(1, "terminate_agent")
            self.env.send_message(2, "terminate_agent")
        time.sleep(1)
        self._master.destroy()

    def show(self):
        self.canvas.delete(tk.ALL)
        self.redraw()


    def redraw(self):
        self.canvas.create_rectangle(0, HEIGHT, WIDTH, 
                                     HEIGHT + TEXT_AREA_HEIGHT,
                                     fill='white', outline='white')

        x,y = DROP_POSITION
        self.canvas.create_oval(x-DROP_RADIUS, y-DROP_RADIUS,
                                x+DROP_RADIUS, y+DROP_RADIUS,
                                fill='red', outline='red')

        for robot in self.env.robots:
            self.drawRobot(robot)
        for bottle in self.env.bottles:
            x,y = int(bottle.x), int(bottle.y)
            self.canvas.create_oval(x-BOTTLE_RADIUS, y-BOTTLE_RADIUS,
                                    x+BOTTLE_RADIUS, y+BOTTLE_RADIUS,
                                    fill=BOTTLE_COLOUR, 
                                    outline=BOTTLE_COLOUR)
            

    def drawRobot(self, robot):
        """Draw the robot"""
        x,y = robot.x, robot.y
        d = robot.dir
        self.canvas.create_oval(x-ROBOT_RADIUS, y-ROBOT_RADIUS,
                                x+ROBOT_RADIUS, y+ROBOT_RADIUS,
                                fill=ROBOT_COLOUR, 
                                   outline=ROBOT_COLOUR)

        self.canvas.create_arc(x-ROBOT_RADIUS, y-ROBOT_RADIUS,
                               x+ROBOT_RADIUS, y+ROBOT_RADIUS,
                               start = 180*(HC_ANGLE - d)/math.pi,
                               extent = -180*2*HC_ANGLE/math.pi,
                               fill=CAMERA_ARC_COLOUR, 
                               outline=CAMERA_ARC_COLOUR)
        wheel1 = [(15,14),(15,10),(7,10),(7,14)]
        wheel2 = [(15,-14),(15,-10),(7,-10),(7,-14)]
        transform(x, y, d, wheel1)
        transform(x, y, d, wheel2)
        self.canvas.create_polygon(*wheel1, fill='black', outline='black')
        self.canvas.create_polygon(*wheel2, fill='black', outline='black')
        backwheel = [(-16, 0)]
        transform(x, y, d, backwheel)
        cx,cy = backwheel[0]
        self.canvas.create_oval(cx-4, cy-4, cx+4, cy+4, 
                                fill='black', outline='black')
        
        if robot.ID == 1:
            colour = ROBOT1_COLOUR
            text_colour=ROBOT1_COLOUR
        else:
            colour = ROBOT2_COLOUR
            text_colour=ROBOT2_COLOUR

        self.canvas.create_oval(x-ROBOT_ID_RADIUS, y-ROBOT_ID_RADIUS, 
                                x+ROBOT_ID_RADIUS, y+ROBOT_ID_RADIUS, 
                                fill=colour, outline=colour)

        if TWO_TASK:
            if robot.ID == 1:
                self.canvas.create_text(10+(robot.ID - 1)*WIDTH/2, HEIGHT + 10,
                                        text=TWO_DISPLAY_TEXT.format(robot.count), font=self.font, fill=text_colour, anchor='nw')
        elif NUM_ROBOTS == 1:    
            #self.canvas.create_text(10+(robot.ID - 1)*WIDTH/2, HEIGHT + 10,
            #                         text=ONE_DISPLAY_TEXT.format(robot.count), font=self.font,fill=text_colour, anchor='nw' )
            pass
        else:
            msg = robot.message.replace("('$none_')", "()")
            self.canvas.create_text(10, HEIGHT + 30*robot.ID - 30,
                                    text=DISPLAY_TEXT.format(robot.count, 
                                                             msg), 
                                    font=self.font, fill=text_colour, 
                                    anchor='nw')
        if robot.holding:
            bottle_pos = [(16, 0)]
            transform(x, y, d, bottle_pos)
            cx,cy = bottle_pos[0]
            self.canvas.create_oval(cx-BOTTLE_RADIUS, cy-BOTTLE_RADIUS,
                                    cx+BOTTLE_RADIUS, cy+BOTTLE_RADIUS,
                                    fill=BOTTLE_COLOUR, 
                                    outline=BOTTLE_COLOUR)
            

    def onKeyPress(self, event):
        key = event.keysym
        if key != 'space':
            return
        if self.running:
            self.canvas.create_text(350,550,text="STOPPED", fill='red')
            if TWO_TASK:
                self.env.send_message(1, "suspend_agent")
            else:
                self.env.send_message(1, "suspend_agent")
                self.env.send_message(2, "suspend_agent")
        else:
            self.show()
            if TWO_TASK:
                self.env.send_message(1, "resume_agent")
            else:
                self.env.send_message(1, "resume_agent")
                self.env.send_message(2, "resume_agent")
        self._master.after(TIME_STEP, self.timer_cb)
        self.env.flipStopped()
        self.running = not self.running

    def timer_cb(self):
        if self.running:
            self.env.step()
            self.show()
            self._master.after(TIME_STEP, self.timer_cb)

    def data_cb(self, event):
        self.env.process_msg(message_queue.get())

    def button_press_cb(self, event):
        self.moving_bottle = self.env.getBottle(event.x, event.y)

    def button_press_add_cb(self, event):
        self.env.addBottle(event.x, event.y)

    def button_press_rem_cb(self, event):
        self.env.removeBottle(event.x, event.y)

    def button_release_cb(self, event):
        self.moving_bottle = None

    def motion_cb(self, event):
        if self.moving_bottle is not None:
            self.moving_bottle.move(event.x, event.y)

    def mid_button_press_cb(self, event):
        self.env.addBottle(event.x, event.y)

    def right_button_press_cb(self, event):
        self.env.removeBottle(event.x, event.y)



if __name__ == "__main__":
    if len(sys.argv) == 2:
        if sys.argv[1].lower() == "twotasks":
            TWO_TASK = True
        elif sys.argv[1].lower() == "one":
            NUM_ROBOTS = 1
    root = tk.Tk()
    BottlesApp(root)
    root.mainloop()

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
import argparse

IS_VERSION_2 = sys.version_info < (3, 0)
if IS_VERSION_2:
    import Queue
    import Tkinter as tk
    import tkMessageBox
    import tkFont
else:
    import queue as Queue
    import tkinter as tk
    from tkinter import messagebox as tkMessageBox
    from tkinter import font as tkFont

message_queue = Queue.Queue()

NUM_ROBOTS = 2
CHARGE_CYCLE = 40
ACTIVITY_CYCLE = 20
BATTERY_CHARGE_FULL = 40 #100 #500
CHARGING_DELTA = 2.1
DISCHARGING_DELTA = 0.5

black = 0, 0, 0
grey = 100,100,100
red = 255, 0, 0
blue = 0, 0, 255


#ACTIVITY_ROOMS = [0,3,4]
ACTIVITY_ROOMS = [0,3,5]  

ROBOT_COLOURS = ['#0000DD', '#b000b0', '#f46747']
ROBOT_ACTIVITY_COLOURS = ['#ddddff', '#fdddfd', '#fbcabf']

WHITE = '#ffffff'

WIDTH = 740           # screen width
HEIGHT = 740          # screen height
TEXT_WIDTH = 60
ROBOT_RADIUS = 10

SQUARE_SIZE = 4

SPEED_FACTOR = 3         # forward speed
RATE_FACTOR = 2
SPEED_UP_FACTOR = 2       # Mult factor for speed change away from centre
SQ_SLOW_DOWN_DIST = 900  # square of dist to centre for speed change

WALL_WIDTH = 4

DOOR_WIDTH = 48
ROOM_SIZE = WIDTH//SQUARE_SIZE
SEE_DIST_SQ = (ROOM_SIZE * 0.66)**2
TEXT_OFFSET = 3*ROOM_SIZE//8

COL1 = '#AA0000'
COL2 = '#AAAA00'
COL3 = '#00AA00'
COL4 = '#00AAAA'

CENTRE_LINES = [r*ROOM_SIZE + ROOM_SIZE//2 for r in range(SQUARE_SIZE)]

DOOR_COLOUR_DICT = {"blue":"#0000FF",
                    "green":"#00AA00",
                    "red":"#FF0000",
                    "yellow":"#E0E000"}
                    
DOORS0 = {(0,1): "blue",
         (2,3): "blue",
         #(0,4): "green",
         (1,5): "green",
         (2,6): "green",
         (5,6): "blue",
         (9,10): "blue",
         (4,5): "yellow",
         (8,9): "yellow",
         (4,8): "red",
         (8,12): "green",
         (3,7): "red",
         (6,7): "yellow",
         (10,11): "green",
         (5,9): "red",
         (6,10): "red",
         (9,13): "green",
         (10,14): "yellow",
         (11,15): "red",
         (12,13): "blue",
         (14,15): "blue"}


DOOR_STATUS = { (r1, r2) : 1 for r1, r2 in DOORS0}

DOORS = {(r1,r2):DOORS0[(r1, r2)] for r1, r2 in DOORS0}
DOORS.update({(r2,r1):DOORS0[(r1, r2)] for r1, r2 in DOORS0})

EDGES = { (r1, DOORS[(r1, r2)]):r2 for (r1,r2) in DOORS}


ROOM_CENTRES = [(ROOM_SIZE*c + ROOM_SIZE//2, ROOM_SIZE*r + ROOM_SIZE//2) \
                for r in range(SQUARE_SIZE) for c in range(SQUARE_SIZE)]

FLOOR_LINES = [(ROOM_CENTRES[r1], ROOM_CENTRES[r2]) for r1,r2 in DOORS0]

CHARGER_RADIUS = 15
CENTRE_RADIUS = 2
CHARGER_POSITIONS = [(ROOM_SIZE//2 - CHARGER_RADIUS*2 - 2,
                      3*ROOM_SIZE + ROOM_SIZE//2),
                     (3*ROOM_SIZE + ROOM_SIZE//2 + CHARGER_RADIUS*2 + 2,
                      3*ROOM_SIZE + ROOM_SIZE//2)]

CAMERA_ANGLE = 45   # the camera vision angle
HC_ANGLE = CAMERA_ANGLE//2
CAMERA_ARC_COLOUR = '#C8C800'

NEW_AGENT_TXT = "New Robot {0}:\nchoose (click on) initial activity room"
ACTIVITY_AGENT_TXT = "Current Agent navigator{0}:\nchoose (click on) new activity room"

TURN_SPEED = 2
MOVE_SPEED = 2

INIT_ROBOT_DIR = [0, 90, 90]

SEE_CENTRE_CLOSE_DIST = 2*ROBOT_RADIUS + 8

INIT_ROBOT_POS = [(ROOM_SIZE//2 - SEE_CENTRE_CLOSE_DIST, ROOM_SIZE//2),
                  (ROOM_SIZE*3 + ROOM_SIZE//2,
                   ROOM_SIZE//2 - SEE_CENTRE_CLOSE_DIST),
                  (ROOM_SIZE + ROOM_SIZE//2,
                   ROOM_SIZE + ROOM_SIZE//2 - SEE_CENTRE_CLOSE_DIST)]


MANUAL = False
NO_REPLAN = False

def door_direction(room1, room2) :
    r1,c1 = room1//SQUARE_SIZE, room1 % SQUARE_SIZE
    r2,c2 = room2//SQUARE_SIZE, room2 % SQUARE_SIZE
    if r1 == r2:
        if c1 > c2:
            return "west"
        else:
            return "east"
    elif c1 == c2:
        if r1 > r2:
            return "north"
        else:
            return "south"
    else:
        return None

DIRECTION_FROM_ROOM_TO_ROOM  = {(r1, r2):door_direction(r1,r2) \
                        for r1 in range(SQUARE_SIZE**2) \
                        for r2 in range(SQUARE_SIZE**2)}



ANGLE_DELTA = 5

ADJACENT = [-1, 1, -SQUARE_SIZE, SQUARE_SIZE]

def centre_point(r1, r2):
    c1x, c1y = ROOM_CENTRES[r1]
    c2x, c2y = ROOM_CENTRES[r2]
    return ((c1x+c2x)//2, (c1y + c2y)//2)
    
DOOR_CENTRES = {(r1,r2):centre_point(r1, r2) for (r1, r2) in DOORS}

ANGLE_NESW = {90:"east", 180:"south", 270:"west", 0:"north"}

DOORWAY_DIST = 2*ROBOT_RADIUS - 3


def sign(x) :
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
    
def xy2room(x, y):
    return (int(y+0.5)//ROOM_SIZE)*SQUARE_SIZE + (int(x+0.5)//ROOM_SIZE)

def get_axis(dir):
    if dir < 0: dir += 360
    if dir < ANGLE_DELTA or dir > 360 - ANGLE_DELTA:
        return 0
    if 90-ANGLE_DELTA < dir < 90+ANGLE_DELTA:
        return 90
    if 180-ANGLE_DELTA < dir < 180+ANGLE_DELTA:
        return 180
    if 270-ANGLE_DELTA < dir < 270+ANGLE_DELTA:
        return 270
    return None

AXIS_NSEW = {
    0:    'east',
    90:   'south',
    180:  'west',
    270:  'north'
    }

AXIS_RAIL = {
    0:    'ew',
    90:   'ns',
    180:  'ew',
    270:  'ns'
    }

REV_DIR = {
    "north" : "south",
    "south" : "north",
    "east" : "west",
    "west" : "east"
    }

def rev_dir(angle):
    return REV_DIR[angle]

def close_to_door_wall(r1, r2, x, y):
    cx, cy = DOOR_CENTRES[(r1, r2)]
    if abs(r1 - r2) == 1:
        return (abs(cx - x) < 15) and (abs(cy - y) < ROOM_SIZE/3)
    else:
        return (abs(cy - y) < 15) and (abs(cx - x) < ROOM_SIZE/3)
    

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
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        """Run as thread - post an event when notifications arrive"""
    
        while (self.running):
            try:
                msg = self.client.get_term()
                p2pmsg = msg[0]
                message_queue.put(p2pmsg)
                self.master.event_generate("<<pedro>>")
            except Exception(ex):
                print("Exception", ex)
                break

    def stop(self):
        self.running = False
    
class ClickMsg(tk.Toplevel):
    def __init__(self, parent, text) :
        tk.Toplevel.__init__(self)
        x = parent._master.winfo_x()
        y = parent._master.winfo_y()
        self.geometry("300x80+{0}+{1}".format(x+WIDTH+190, y)) #y+HEIGHT//2))
        self.overrideredirect(True)
        self.lift()
        self.text = tk.Text(self, bg="orange")
        self.text.pack(fill=tk.BOTH, expand=1)
        self.text.insert(tk.END, text)
        self.text.see(tk.END)


class Robot(object):
    def __init__(self, env, ID):
        self.x,self.y = -50, -50
        self.activity_room = None
        self.dir = INIT_ROBOT_DIR[ID-1]
        self.turning_speed = 0
        self.vx, self.vy = 0, 0
        self.id = ID
        self.env = env
        self.stopped = False
        self.queued_messages = []
        self.saved_room = -5
        self.saved_room_position = None  # 'at' 'near' 'far'
        self.saved_room_side = None  # 'north' etc
        self.saved_angle = 'north'
        #self.saved_room_centre = False
        #self.saved_see_room_centre = False
        #self.saved_see_centre_close = False
        #self.saved_pointing = None
        #self.saved_through_doorway = None
        #self.saved_pointing = None
        self.started = False
        self.saved_battery_status_high = True
        self.saved_on_line = None
        self.path = []
        self.turn_stop = None
        self.discharge_cycle = 0
        self.battery_charge = BATTERY_CHARGE_FULL

    def set_path(self, path):
        points = [ROOM_CENTRES[r-1] for  r in path]
        points.reverse()
        if points:
            points.pop()
        self.path = points
        
    def turn(self, rail):
        dir = self.dir
        self.turn_stop = rail
        if rail == 'ew':
            if 0 <= dir <= 90:
                sign = -1
            elif 90 < dir <= 180:
                sign = 1
            if 180 < dir <= 270:
                sign = -1
            elif 270 < dir < 360:
                sign = 1
        else:
            if 0 <= dir <= 90:
                sign = 1
            elif 90 < dir <= 180:
                sign = -1
            if 180 < dir <= 270:
                sign = 1
            elif 270 < dir < 360:
                sign = -1

        self.turning_speed = sign*TURN_SPEED

        
    def door_on_path(self):
        dx = int(100*round(math.cos(math.pi*self.dir/180)))
        dy = int(100*round(math.sin(math.pi*self.dir/180)))
        return DOORS.get((xy2room(self.x - dx, self.y - dy),
                          xy2room(self.x + dx, self.y + dy)))
        
        
    def get_room(self):
        return xy2room(self.x, self.y)

    def close_to_room_centre(self):
        cx, cy =  ROOM_CENTRES[self.get_room()]
        return (self.x - cx)**2 + (self.y - cy)**2 < 5
    
    def close_doorway(self):
        doorx = int((float(self.x) / ROOM_SIZE) + 0.5) * ROOM_SIZE
        doory = int((float(self.y) / ROOM_SIZE) + 0.5) * ROOM_SIZE
        if self.dir == 0:
            return 0 < doorx - self.x < DOORWAY_DIST
        if self.dir == 90:
            return 0 < doory - self.y < DOORWAY_DIST
        if self.dir == 180:
            return 0 < self.x - doorx  < DOORWAY_DIST
        if self.dir == 270:
            return 0 < self.y - doory  < DOORWAY_DIST

    def about_to_enter(self, room):
        if not self.close_doorway():
            return False
        dx = int(DOORWAY_DIST*round(math.cos(math.pi*self.dir/180)))
        dy = int(DOORWAY_DIST*round(math.sin(math.pi*self.dir/180)))
        return room == xy2room(self.x + dx, self.y + dy)
        
    def through_doorway(self):
        doorx = int((float(self.x) / ROOM_SIZE) + 0.5) * ROOM_SIZE
        doory = int((float(self.y) / ROOM_SIZE) + 0.5) * ROOM_SIZE
        if self.dir == 0:
            return 0 <= self.x - doorx  < DOORWAY_DIST
        if self.dir == 90:
            return 0 <= self.y - doory  < DOORWAY_DIST
        if self.dir == 180:
            return 0 <= doorx - self.x < DOORWAY_DIST
        if self.dir == 270:
            return 0 <= doory - self.y < DOORWAY_DIST
    
    def move(self, side):
        dir = get_axis(self.dir)
        if dir is not None:
            self.dir = dir
        else:
            print("PATH ERROR !!!!! {} {} {} {}".format(self.dir, self.x, self.y, self.get_room()))
        if side == 'east':
            if dir == 0:
                sign = 1
            elif dir == 180:
                sign = -1
            else:
               print("RAIL ERROR !!!!! {} {} {} {}".format(self.dir, self.x, self.y, self.get_room()))
        elif side == 'west':
            if dir == 0:
                sign = -1
            elif dir == 180:
                sign = 1
            else:
               print("RAIL ERROR !!!!! {} {} {} {}".format(self.dir, self.x, self.y, self.get_room()))
               
        elif side == 'north':
            if dir == 270:
                sign = 1
            elif dir == 90:
                sign = -1
            else:
               print("RAIL ERROR !!!!! {} {} {} {}".format(self.dir, self.x, self.y, self.get_room()))
        elif side == 'south':
            if dir == 270:
                sign = -1
            elif dir == 90:
                sign = 1
            else:
               print("RAIL ERROR !!!!! {} {} {} {}".format(self.dir, self.x, self.y, self.get_room()))

               
        self.vx = int(sign*MOVE_SPEED*round(math.cos(math.pi*self.dir/180)))
        self.vy = int(sign*MOVE_SPEED*round(math.sin(math.pi*self.dir/180)))
        
    def can_see_room_centre(self):
        angle = get_axis(self.dir)
        if angle is None:
            return False
        cx, cy = ROOM_CENTRES[self.get_room()]
        angle1 = get_axis(180*math.atan2((cy - self.y), (cx - self.x))/math.pi)
        dist = math.sqrt((self.x - cx)**2 + (self.y - cy)**2)
        return (dist > 0.1) and  (angle == angle1)

    def get_full_path(self):
        points = self.path[:]
        if points == []:
            return []
        ocx,ocy = points[-1]
        # if the last point is in different room and not in segment between
        # this rooms centre and next room centre
        if self.get_room() != xy2room(ocx,ocy):
            cx,cy = ROOM_CENTRES[self.get_room()]
            if (sign(cx - self.x) != sign(cx - ocx)) or \
               (sign(cy - self.y) != sign(cy - ocy)):
                points.append((cx,cy))
        points.append((self.x, self.y))
        return points

    def is_charging(self):
        room = self.get_room()
        if room == 12:
            x,y = CHARGER_POSITIONS[0]
            return abs(x - self.x) < 6 and abs(y - self.y) < 4
        if room == 15:
            x,y = CHARGER_POSITIONS[1]
            return abs(x - self.x) < 6 and abs(y - self.y) < 4
        return False
            
    def stop(self):
        self.turning_speed = 0
        self.vx, self.vy = 0, 0
        if self.close_to_room_centre():
            self.x, self.y = ROOM_CENTRES[self.get_room()]

    def doing_activity(self):
        return self.vx == self.vy == self.turning_speed and\
            self.get_room() == self.activity_room and\
            self.saved_room_position == 'at'

    def update(self):
        if self.stopped:
            return
        room = self.get_room()
        if not MANUAL:
            if self.doing_activity():
                if self.battery_charge <= 0:
                    self.battery_charge = 0
                    if self.saved_battery_status_high:
                        self.add_message("u_(battery_status(low))")
                        self.saved_battery_status_high = False
                else:
                    self.battery_charge -= DISCHARGING_DELTA
            elif self.is_charging():
                if not self.saved_battery_status_high:
                    if room == 12:
                        self.x,self.y = CHARGER_POSITIONS[0]
                    elif room == 15:
                        self.x,self.y = CHARGER_POSITIONS[1]
                if self.battery_charge >= BATTERY_CHARGE_FULL:
                    self.battery_charge = BATTERY_CHARGE_FULL
                    if not self.saved_battery_status_high:
                        self.add_message("u_(battery_status(high))")
                        self.saved_battery_status_high = True
                else:
                    self.battery_charge += CHARGING_DELTA
            else:
                self.discharge_cycle = 0
                
        cx, cy =  ROOM_CENTRES[room]
        if not self.started:
            return
        if (self.x - cx)**2 + (self.y - cy)**2 < 900:
            vx = self.vx
            vy = self.vy
        else:
            vx = SPEED_UP_FACTOR*self.vx
            vy = SPEED_UP_FACTOR*self.vy
        self.x += vx
        self.y += vy
        self.dir += self.turning_speed
        if self.dir > 360:
            self.dir -= 360
        elif self.dir < 0:
            self.dir += 360
        if self.path and self.get_room() == xy2room(*self.path[-1]) and \
           self.close_to_room_centre():
            self.x, self.y =  ROOM_CENTRES[self.get_room()]
            self.path.pop()
        self.check_on_line()
        self.check_position()
        self.send_messages()

    def check_position(self):
        changed = False
        # check for room change
        room = self.get_room() + 1
        if room < 0:
            return
        if room != self.saved_room:
            changed = True
            self.saved_room = room
        # check for change of position and angle within room
        cx, cy = ROOM_CENTRES[self.get_room()]
        dist = math.sqrt((self.x - cx)**2 + (self.y - cy)**2)
        angle1 = get_axis(180*math.atan2((cy - self.y), (cx - self.x))/math.pi)
        if angle1 is None:
            angle = self.saved_angle
        angle = AXIS_NSEW[angle1]
        self.saved_angle = angle
        if (self.x - cx)**2 + (self.y - cy)**2 < 5:
            if self.saved_room_position != 'at':
                self.saved_room_position = 'at'
                changed = True
        elif dist < SEE_CENTRE_CLOSE_DIST:
            if self.saved_room_position != 'near':
                self.saved_room_position = 'near'
                changed = True
        else:
            if self.saved_room_position != 'far':
                self.saved_room_position = 'far'
                changed = True
        if changed:
            self.add_message(f'u_(my_position({self.saved_room}, {rev_dir(self.saved_angle)}, { self.saved_room_position}))')  
                    
           
    def check_on_line(self):
        angle = get_axis(self.dir)
        if angle is None:
            if self.saved_on_line is not None:
                self.add_message('f_(on_line(_))')
                self.saved_on_line = None
            return
        rail = AXIS_RAIL[angle]
        if rail != self.saved_on_line:
            if self.turn_stop == rail:
                self.turning_speed = 0
                self.turn_stop = None
            self.add_message('u_(on_line({0}))'.format(rail))
            self.saved_on_line = rail
                  
    def flipStopped(self):
        if self.stopped:
            self.env.parent.client.p2p('messages:navigator{0}'.format(self.id), 'resume_agent')
        else:
            self.env.parent.client.p2p('messages:navigator{0}'.format(self.id), 'suspend_agent')
        self.stopped = not self.stopped

    def add_message(self, msg):
        self.queued_messages.append(msg)

    def send_msg(self, msg):
        self.env.send_msg(self.id, msg)

    def shutdown(self, pedroclient):
        self.env.parent.client.p2p('messages:navigator{0}'.format(self.id),
                                   'terminate_agent')
        
    def send_messages(self):
        if self.queued_messages:
            print(self.queued_messages, self.id)
            message = '['
            for m in self.queued_messages:
                message += m + ','
            message = message[:-1]+']'
            self.send_msg(message)
            self.queued_messages = []

    def over_robot(self, room, x, y):
        return room == self.get_room() and \
            (x - self.x)**2 + (y -self.y)**2 < 200
    
    def process_click(self, room, x, y):
        if self.over_robot(room, x, y) :
            if room in [12,15] :
                self.battery_charge = BATTERY_CHARGE_FULL
                if not self.saved_battery_status_high:
                    self.add_message("u_(battery_status(high))")
                    self.saved_battery_status_high = True
                self.send_messages()
            elif room == self.activity_room:
                self.battery_charge = 0
                if self.saved_battery_status_high:
                    self.add_message("u_(battery_status(low))")
                    self.saved_battery_status_high = False
                self.send_messages()

    def close_to_door(self, r1, r2):
        cx,cy = DOOR_CENTRES[(r1,r2)]
        return (abs(self.x - cx) < 40) and (abs(self.y - cy) < 40)
            
class Environment(object):
    def __init__(self, parent):
        self.parent = parent

        self.leftdown= False
        self.messages_to_send = None
        robot0 = Robot(self, 1)
        robot1 = Robot(self, 2)
        robot2 = Robot(self, 3)
        self.robots = [robot0, robot1, robot2]

        self.door_status = DOOR_STATUS
        self.task_actions = {}

    def free_room(self, room, is_new):
        if room in [12,15]: return False
        for robot in self.robots:
            if room == robot.activity_room:
                return False
            if is_new and (room == robot.get_room() or\
                           robot.about_to_enter(room)):
                return False
        return True
    
    def shutdown(self, pedroclient):
        for r in self.robots:
            r.shutdown(pedroclient)

    def overlapping_robots(self):
        num = len(self.robots)
        for i in range(num-1):
            ri = self.robots[i]
            if not ri.started:
                return False
            for j in range(i+1, num):
                rj = self.robots[j]
                if not rj.started:
                    return False
                if (ri.x - rj.x)**2 + (ri.y - rj.y)**2 < 2*ROBOT_RADIUS**2:
                    return True
        return False
    
    def step(self):
        for r in self.robots:
            r.update() 
        # This shouldn't happen
        if self.overlapping_robots() and not self.robots[0].stopped:
            print('!!!!!!! Overlapping Robots !!!!!')
            self.flipStopped()
            self.parent.canvas.create_text(330,530,text="ERROR STOPPED", fill='red')

    def flipStopped(self):
        for r in self.robots:
            r.flipStopped()

    def send_msg(self, ID, msg):
        self.parent.client.p2p("percepts:{0}{1}".format(self.sender_process, ID), msg)
            
    def process_msg(self, term):
        if term.type == pedroclient.PObject.structtype and \
           term.functor.val == "p2pmsg":
            msg = term.args[2]
            
            if str(msg) == "initialise_":
                sender_process = term.args[1].args[0].args[1].val
                sender_task = sender_process 
                self.sender_process = sender_process[:-1]
                robotID = int(sender_process[-1])-1
                self.robots[robotID].started = True
                self.robots[robotID].add_message('r_(battery_status(high))')
                self.saved_battery_status_high = True
                self.robots[robotID].update()
                return
            if msg.get_type() == pedroclient.PObject.structtype and \
                 msg.functor.val == 'actions':
                sender_process = term.args[1].args[0].args[1].val
                sender_task = sender_process 

                action_term = msg.args[1]
                if action_term.get_type() == pedroclient.PObject.listtype:
                    actions = action_term.toList()
                else:
                    actions = []
                old_actions = self.task_actions.get(sender_task, [])
                self.task_actions[sender_task] = actions
                (start_actions, stop_actions) \
                    = get_start_stop_actions(old_actions, actions)
                self.process_stop_actions(stop_actions, sender_task)
                self.process_start_actions(start_actions, sender_task)
                return
            else:
                    assert (False), "COMMAND: {}".format(str(msg))
        else:
            message = str(term)
            message = message.replace('\\\\"', '"')
            self.parent.text.configure(state="normal")
            colour = "conflict" if message.startswith("conflict") else "black"
            self.parent.text.insert(tk.END, message+'\n', colour)
            self.parent.text.mark_set("insert",tk.END)
            self.parent.text.see(tk.END)
            self.parent.text.configure(state="disabled")
            if term.get_type() == pedroclient.PObject.structtype and \
               (term.functor.val ==  'path' or\
                term.functor.val ==  'exit_room_path'):
                robotID = int(term.args[0].val[-1])-1
                if term.args[1].get_type() == pedroclient.PObject.listtype:
                    path = [n.val for n in term.args[1].toList()]
                else:
                   path = []
                self.robots[robotID].set_path(path)
                return
            elif term.get_type() == pedroclient.PObject.structtype and \
                 term.functor.val ==  'no_path':
                robotID = int(term.args[0].val[-1])-1
                self.robots[robotID].set_path([])
                return
            elif term.get_type() == pedroclient.PObject.structtype and \
                 term.functor.val ==  'joining':
                robot_handle = term.args[0].val
                robot = int(robot_handle[-1]) - 1
                for r1, r2 in self.door_status:
                    if self.door_status[(r1,r2)] == 0:
                        door_dir = DIRECTION_FROM_ROOM_TO_ROOM[(r1,r2)]
                        msg = "door_status({0}, {1}, shut)".\
                              format(door_dir, r1+1)
                        self.parent.client.p2p("messages:{0}".format(robot_handle), msg)
                if NO_REPLAN:
                    self.parent.client.p2p("messages:{0}".format(robot_handle), "no_replan()")
                self.parent.is_joining = True
                self.parent.robot_change_activity = (self.robots[robot], 1, 0, 0)
                self.display_click(robot_handle)

    def display_click(self, robot_handle):
        self.parent.canvas.create_rectangle(2*ROOM_SIZE+20,8,2*ROOM_SIZE+300, 60, fill="grey80",outline="grey80")
        self.parent.canvas.create_text(520,30,text=NEW_AGENT_TXT.format(robot_handle), fill='red')
        
    def process_stop_actions(self, stop_actions, sender_task):
        robotID = int(sender_task[-1])-1
        for a in stop_actions:
            print("stop_action", sender_task, str(a))
            self.robots[robotID].stop()
            return
        
    def process_start_actions(self, start_actions, sender_task):
        robotID = int(sender_task[-1])-1
        #if start_actions:
        #    self.start_animate()
        for a in start_actions:
            print("start_action", sender_task, str(a))
            cmd_args = a.args
            cmd = a.functor.val
            if cmd == 'turn_to':
                self.robots[robotID].turn(cmd_args[0].val)
            elif cmd == 'move':
                self.robots[robotID].move(cmd_args[0].val)
            else:
                assert (False), "COMMAND: {}".format(str(a))

            
class RoomsApp(object):

    def __init__(self, master):
        self._master = master
        master.title("Rooms")
        master.protocol("WM_DELETE_WINDOW", self.onClose)
        #master.geometry("{0}x{1}".format(WIDTH+WALL_WIDTH+2+TEXT_WIDTH, HEIGHT+WALL_WIDTH))
        self.env = Environment(self)
        self.canvas = tk.Canvas(master, bg = 'white',
                                width = WIDTH+4, height=HEIGHT+4)
        self.canvas.pack(fill=tk.BOTH, expand=1, side=tk.LEFT)
        self.canvas.bind("<Button-1>", self.button_press_cb)
        self.canvas.bind("<Control-Button-1>", self.button_press_add_cb)
        self.canvas.bind("<Shift-Button-1>", self.button_shift_press_cb)
        self.canvas.bind("<ButtonRelease-1>", self.button_release_cb)
        self.canvas.bind("<Button-2>", self.mid_button_press_cb)
        self.canvas.bind("<Button-3>", self.right_button_press_cb)
        self.canvas.bind("<B1-Motion>", self.motion_cb)
        self._master.bind("<<pedro>>", self.data_cb)
        scrollbar = tk.Scrollbar(master)
        scrollbar.pack(side=tk.RIGHT,fill=tk.Y)
        self.text = tk.Text(master,
                            yscrollcommand=scrollbar.set,
                            width=TEXT_WIDTH)
        self.text.tag_config('conflict', foreground="orange")
        self.text.tag_config('black', foreground="black")
        
        self.text.pack(fill=tk.BOTH, expand=1, side=tk.LEFT)
        scrollbar.config(command=self.text.yview)
        self.canvas.bind_all("<Key>", self.onKeyPress)
        self.text.configure(state="disabled")
        self.text.bind("<1>", lambda event: self.text.focus_set())
        #self.font = tkFont.Font(family=FONT_NAME,size=FONT_SIZE,weight="bold")
        self.running = True
        self.client = pedroclient.PedroClient()
        self.client.register('rooms_env')
        self.client.subscribe("path(X,Y,Z)", "true")
        self.client.subscribe("door_status(X,Y,Z)", "true")
        self.client.subscribe("release(X,Y)", "true")
        self.client.subscribe("new_activity_room(X,Y)", "true")
        #self.client.subscribe("position_msg(Rob, Rm, Dir, Dist)", "true")
        self.client.subscribe("new_loc(Rob, Rm)", "true")
        self.client.subscribe("room_side(Rob, Dir)", "true")
        self.client.subscribe("end_room_side(Rob)", "true")
        self.client.subscribe("no_path(Rob)", "true")
        self.client.subscribe("joining(Rob)", "true")
        self.client.subscribe("reserve_room(Rob, Rm, Path)", "true")
        self.client.subscribe("conflict_message(Conflict)", "true")
        self.client.subscribe("book_centre(Rob, Rm)", "true")
        self.client.subscribe("cancel_book_centre(Rob, Rm)", "true")
        self.client.subscribe("book_door(Rob, Rm1, Rm2)", "true")
        self.client.subscribe("cancel_book_door(Rob)", "true")
        
        self.thread = GetNotifications(self.client, master)
        self.thread.start()
        self.moving_bottle = None
        self.robot_change_activity = None
        self.charging_cycle = 0
        self.charging_cycle_dir = 1
        self.show()
        self.is_joining = False
        self._master.after(30, self.timer_cb)

    def onClose(self):
        self.env.shutdown(self.client)
        time.sleep(1)
        self._master.destroy()

    def show(self):
        self.canvas.delete(tk.ALL)
        self.redraw()


    def redraw(self):
        self.charging_cycle += self.charging_cycle_dir
        if self.charging_cycle_dir == 1 and\
           self.charging_cycle == CHARGE_CYCLE:
            self.charging_cycle_dir = -1
        elif self.charging_cycle_dir == -1 and\
             self.charging_cycle == 0:
            self.charging_cycle_dir = 1
        self.draw_activity_rooms()
        self.draw_room_nums()
        self.draw_walls()
        self.draw_floor_lines()
        self.draw_doors()
        self.draw_chargers()
        
        for robot in self.env.robots:
            self.drawRobotPath(robot)
        for robot in self.env.robots:
            self.drawRobot(robot)

    def charging(self, i):
        x,y = CHARGER_POSITIONS[i]
        for robot in self.env.robots:
            if abs(x - robot.x) < 3 and abs(y - robot.y) < 3:
                return True
        return False
            
    def draw_activity_rooms(self):
        for robot in self.env.robots:
            if robot.activity_room is not None:
                room = robot.activity_room
                r,c = room % SQUARE_SIZE, room // SQUARE_SIZE
                r *= ROOM_SIZE
                c *= ROOM_SIZE
                colour = ROBOT_ACTIVITY_COLOURS[robot.id-1]
                self.canvas.create_rectangle(r,c, r+ROOM_SIZE, c+ROOM_SIZE,
                                             fill=colour, outline=colour)

    def draw_room_nums(self):
        for i, (cx, cy) in enumerate(ROOM_CENTRES):
            cx -= TEXT_OFFSET
            cy -= TEXT_OFFSET
            self.canvas.create_text(cx, cy, text=str(i+1)) 
        
    def draw_floor_lines(self):
        for cx,cy in ROOM_CENTRES:
            self.canvas.create_line((cx-2*ROBOT_RADIUS-5, cy),
                                    (cx+2*ROBOT_RADIUS+5, cy), fill='black')
            self.canvas.create_line((cx, cy-2*ROBOT_RADIUS-5),
                                    (cx, cy+2*ROBOT_RADIUS+5), fill='black')

             
        for p1,p2 in FLOOR_LINES:
            self.canvas.create_line(p1,p2, fill='black')

    def draw_walls(self):
        
        
        for r in range(SQUARE_SIZE+1):
            self.canvas.create_rectangle(r*ROOM_SIZE, 0,  
                                         r*ROOM_SIZE+WALL_WIDTH,
                                         HEIGHT+WALL_WIDTH,
                                         fill='black', outline='black')
        for r in range(SQUARE_SIZE+1):
            self.canvas.create_rectangle(0, r*ROOM_SIZE,   
                                         WIDTH+WALL_WIDTH,
                                         r*ROOM_SIZE + WALL_WIDTH,
                                         fill='black', outline='black')

        for (r1, r2) in DOORS0:
            r,c = r2 % SQUARE_SIZE, r2 // SQUARE_SIZE
            if r2 -r1 == 1:
                dt = c*ROOM_SIZE + ROOM_SIZE//2
                self.canvas.create_rectangle(r*ROOM_SIZE,
                                             dt - DOOR_WIDTH//2,  
                                             r*ROOM_SIZE+WALL_WIDTH,
                                             dt + DOOR_WIDTH//2,
                                             fill='white', outline='white')
            else:
                dt = r*ROOM_SIZE + ROOM_SIZE//2
                self.canvas.create_rectangle(dt - DOOR_WIDTH//2,
                                             c*ROOM_SIZE, 
                                             dt + DOOR_WIDTH//2,
                                             c*ROOM_SIZE+WALL_WIDTH,
                                             fill='white', outline='white')


                
    def draw_doors(self):
        for (r1, r2) in DOORS0:
            colour_name = DOORS0[(r1, r2)]
            rd = r2-r1
            colour = DOOR_COLOUR_DICT[colour_name]
            if rd == 1:
                # x-adjacent
                r,c = r2 % SQUARE_SIZE, r2 // SQUARE_SIZE
                dt = c*ROOM_SIZE + ROOM_SIZE//2
                if self.env.door_status[(r1, r2)] == 0:
                    self.canvas.create_rectangle(r*ROOM_SIZE-2,
                                                 dt - DOOR_WIDTH//2,  
                                                 r*ROOM_SIZE+WALL_WIDTH+2,
                                                 dt + DOOR_WIDTH//2,
                                                 fill=colour,
                                                 outline=colour)
                else:
                    self.canvas.create_rectangle(r*ROOM_SIZE-2,
                                                 dt + DOOR_WIDTH//2,  
                                                 r*ROOM_SIZE+WALL_WIDTH+2,
                                                 dt + 3*DOOR_WIDTH//2,
                                                 fill=colour, outline=colour)

                
            else:
                r,c = r2 % SQUARE_SIZE, r2 // SQUARE_SIZE
                dt = r*ROOM_SIZE + ROOM_SIZE//2
                if self.env.door_status[(r1, r2)] == 0:
                    self.canvas.create_rectangle(dt - DOOR_WIDTH//2,
                                                 c*ROOM_SIZE-2, 
                                                 dt + DOOR_WIDTH//2,
                                                 c*ROOM_SIZE+WALL_WIDTH+2,
                                                 fill=colour, outline=colour)
                else:
                    self.canvas.create_rectangle(dt + DOOR_WIDTH//2,
                                                 c*ROOM_SIZE-2, 
                                                 dt + 3*DOOR_WIDTH//2,
                                                 c*ROOM_SIZE+WALL_WIDTH+2,
                                                 fill=colour, outline=colour)

    def draw_chargers(self):
        for i,(x,y) in enumerate(CHARGER_POSITIONS):
            if self.charging(i):
                outline= 'cyan'
                width = 2 + self.charging_cycle//8
            else:
                outline= 'orange'
                width = 1
            self.canvas.create_oval(x-CHARGER_RADIUS, y-CHARGER_RADIUS,
                                    x+CHARGER_RADIUS, y+CHARGER_RADIUS,
                                    width=width,
                                    fill='orange', 
                                    outline=outline)

    def drawRobotPath(self, robot):    
        path = robot.get_full_path()
        colour = ROBOT_COLOURS[robot.id-1]
        if path:
            self.canvas.create_line(*path, fill=colour,
                                    width=3,dash=(3,3))
    def drawRobot(self, robot):
        """Draw the robot"""
        x,y = robot.x, robot.y
        d = robot.dir
        colour = ROBOT_COLOURS[robot.id-1]
        self.canvas.create_oval(x-ROBOT_RADIUS, y-ROBOT_RADIUS,
                                x+ROBOT_RADIUS, y+ROBOT_RADIUS,
                                fill=colour, 
                                outline=colour)
        xd = ROBOT_RADIUS*math.cos(d*math.pi/180)
        yd = ROBOT_RADIUS*math.sin(d*math.pi/180)
        self.canvas.create_line((x-xd, y-yd), (x+xd, y+yd), fill='white', width=3)

         
        if robot.doing_activity():
            robot.discharge_cycle += 1
            if robot.discharge_cycle == ACTIVITY_CYCLE:
                robot.discharge_cycle = 1
            radius = (ROBOT_RADIUS*robot.battery_charge/BATTERY_CHARGE_FULL) *\
                (robot.discharge_cycle/ACTIVITY_CYCLE)
            self.canvas.create_oval(x-radius, y-radius,
                                    x+radius, y+radius,
                                    fill=WHITE, 
                                    outline=WHITE)
        else:
            radius = ROBOT_RADIUS*robot.battery_charge/BATTERY_CHARGE_FULL
            self.canvas.create_oval(x-radius, y-radius,
                                    x+radius, y+radius,
                                    fill=None, 
                                    outline=WHITE)
        
    def onKeyPress(self, event):
        #self.text.tag_remove(tk.SEL, "1.0", tk.END)
        key = event.keysym
        if key != 'space':
            return 
        if self.running:
            self.canvas.create_text(330,530,text="STOPPED", fill='red')
        else:
            self.show()
            self._master.after(30, self.timer_cb)
        self.env.flipStopped()
        self.running = not self.running

    def timer_cb(self):
        if self.running and not self.robot_change_activity:
            self.env.step()
            self.show()
            self._master.after(30, self.timer_cb)

    def data_cb(self, event):
        self.env.process_msg(message_queue.get())

    def update_new_loc(self, room):
        robot,robot_room, x,y = self.robot_change_activity

        if robot.activity_room is not None:
            robot.process_click(robot_room, x, y)
        else:
            robot.x, robot.y = ROOM_CENTRES[room]

        robot.check_position()
        robot.send_messages()
        self.client.notify("new_activity_room(navigator{0}, {1})".format(robot.id, room+1))
        self.is_joining = False
        robot.activity_room = room
        self.robot_change_activity = None
        self.timer_cb()
                            
    def button_press_cb(self, event):
        room = xy2room(event.x, event.y)
        if self.robot_change_activity and self.env.free_room(room, self.is_joining):
            robot,_robot_room, _x,_y = self.robot_change_activity
            self.update_new_loc(room)
            #if not robot.activity_room:
            #    robot.check_position()
            #    robot.send_messages()


            return
        
        for r1,r2 in DOORS0:
            if close_to_door_wall(r1, r2, event.x, event.y):
                close_robot = False
                for robot in self.env.robots:
                    if robot.close_to_door(r1,r2):
                        close_robot = True
                if not close_robot:
                    self.env.door_status[(r1,r2)] = 1 - self.env.door_status[(r1,r2)]
                    door_status =  "open" if self.env.door_status[(r1,r2)] else "shut"
                    door_dir = DIRECTION_FROM_ROOM_TO_ROOM[(r1,r2)]
                    self.client.notify("door_status({0}, {1}, {2})".\
                                       format(door_dir, r1+1, door_status))
                #return
        if room in [12, 15]:
            for robot in self.env.robots:
                if room == robot.get_room():
                    robot.process_click(room, event.x, event.y)
                    #return
        for robot in self.env.robots:
            if room == robot.activity_room:
                robot.process_click(room, event.x, event.y)
                #return
            

    def button_shift_press_cb(self, event):
        self.button_shift_press(event.x, event.y)

    def button_shift_press(self, x, y):
        room = xy2room(x, y)
        if self.robot_change_activity and self.env.free_room(room, self.is_joining):
            self.update_new_loc(room)
           
        for robot in self.env.robots:
            if robot.over_robot(room, x, y) and room in [12,15]:
                handle = "navigator{0}".format(robot.id)
                self.env.display_click(handle)
                self.robot_change_activity = (robot, room, x, y)

        
        
    def button_press_add_cb(self, event):
        pass

    def button_release_cb(self, event):
        pass

    def motion_cb(self, event):
        pass

    def mid_button_press_cb(self, event):
        pass

    def right_button_press_cb(self, event):
        pass



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--manual", action='store_true', 
                        help="A click on the robot is needed to flip the battery status")
    parser.add_argument("-n", "--no_replan", action='store_true',
                        help="The robot does not replan in order to find a shorter path when doors are opened")
    args = parser.parse_args()
    MANUAL = args.manual
    NO_REPLAN = args.no_replan
    root = tk.Tk()
    RoomsApp(root)
    root.mainloop()


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



import sys
import pedroclient
import time
import copy
import threading

IS_VERSION_2 = sys.version_info < (3, 0)
if IS_VERSION_2:
    import Queue
    import Tkinter as tk
else:
    import queue as Queue
    import tkinter as tk    
message_queue = Queue.Queue()

HEIGHT = 420
NUM_SLOTS_PER_SECTION = 12
NUM_SLOTS = 3*NUM_SLOTS_PER_SECTION   

BLOCK_SIZE = 20

BLOCK_SIZE2 = BLOCK_SIZE//2
ARM_SIZE = 2
TABLE_WIDTH = 6
ARM_SIZE2 = ARM_SIZE//2
BLOCK_SIZE4 = BLOCK_SIZE//4
TABLE_POS = HEIGHT - 70 

BS32 = BLOCK_SIZE2 + BLOCK_SIZE
BS34 = (BLOCK_SIZE*2)//3

WIDTH = (NUM_SLOTS+2)*BS32

TABLE_SIZE = (NUM_SLOTS+2)*BS32


TOP = 100

def slot2pos(i):
    return BS32*(i)

def pos2slot(p):
    return int(0.5 + p/float(BS32))

def pos2slotpos(p):
    if p > TABLE_POS:
        return 0
    return int((TABLE_POS - p)/float(BLOCK_SIZE))

def slotpos2pos(sp):
    return TABLE_POS - (sp+1)*BLOCK_SIZE


TABLE_START = slot2pos(1) #- BLOCK_SIZE4
TABLE_START_LAST = slot2pos(25) #- BLOCK_SIZE4

BLOCKS = [(1, 1, '#c81e1e'),
          (3, 2, '#c8961e'), 
          (4, 3, '#c8b428'), 
          (6, 4, '#c8c828'), 
          (7, 14, '#c8dc3c'),
          (16, 5, '#a0c828'),
          (18, 6, '#32c828'), 
          (20, 7, '#28c8aa'), 
          (21, 8, '#28648c'), 
          (22, 9, '#5050b4'), 
          (23, 15, '#5050f0'), 
          (NUM_SLOTS-7, 16, '#6464ff'), 
          (NUM_SLOTS-8, 10, '#6450f0'), 
          (NUM_SLOTS-5, 11, '#8c28c8'), 
          (NUM_SLOTS-3, 12, '#c828c8'), 
          (NUM_SLOTS-2, 13, '#fa28c8')] 



# arm speed info
SPEED = 20          # initial speed
SPEED_MIN = 5       # minimum speed
SPEED_MAX = 30      # maximum speed
SPEED_DELTA = 1     # change of speed on each left/right arrow press

TASK_COLOURS = ['#646464', 
                '#ff7000', 
                '#fa28c8',
                '#0000ff', 
                '#28c8c8', 
                '#c828c8', 
                '#0000ff', 
                '#009696', 
                '#009696', 
                '#646464'] 


# Whenever an action is started a list of commands is generated that
# replaces the old list of commands and so it is not necessary to
# stop an action a new action is to be started and so we only need to stop
# an action if there are no new actions
def get_start_stop_actions(old_actions, new_actions):
    if new_actions:
        return ([a for a in new_actions if not a in old_actions], [])
    else:
        return ([], old_actions)


def make_fun_0(fun):
    return lambda : fun()

def make_fun_1(fun, arg):
    return lambda : fun(arg)
 
class Pedro(threading.Thread):
    def __init__(self, app):
        self.running = True
        self.app = app
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        while (self.running):
            p2pmsg = self.app.client.get_term()[0]
            message_queue.put(p2pmsg)
            self.app._master.event_generate("<<pedro>>")

    def stop(self):
        self.running = False



class Block(object):
    def __init__(self, x, y, i, c):
        self.x = x
        self.y = y
        self.i = i
        self.istr = str(i)
        self.c = c

    def move(self, dx, dy):
        self.x += dx
        self.y += dy

    def move_abs(self, x, y):
        self.x = x
        self.y = y
        

class Arm(object):
    def __init__(self, i, x, y, app):
        self.x = x
        self.y = y
        self.i = i
        self.app = app
        self.closed = False
        self.holding = None
        self.xspeed = 0
        self.yspeed = 0
        self.commands = []
        self.xdest = None
        self.ydest = None
        self.active = False
        self.lifting = False
        self.colour = 0
        self.speed = SPEED
 
    def slow(self):
        self.speed -= SPEED_DELTA
        if self.speed < SPEED_MIN:
            self.speed = SPEED_MIN

    def quick(self):
        self.speed += SPEED_DELTA
        if self.speed > SPEED_MAX:
            self.speed = SPEED_MAX
    



    def __str__(self):
        return "arm {0}".format(self.i)


    def set_commands(self, commands):
        self.commands = commands
        self.active = True
        if self.commands:
            self.commands.pop(0)()

    def stop(self):
        self.lift()
        self.active = False
        self.colour = 0
        self.commands = []

    def reached_dest(self, current, dest, direction):
        if direction > 0:
            return current > dest - 5
        else:
            return current < dest + 5

    def step(self):
        if self.is_animating():
            self.move(self.xspeed, self.yspeed)
            dx, dy = 0, 0
            yd = self.ydest
            if self.reached_dest(self.x, self.xdest, self.xspeed):
                dx = self.xdest - self.x 
                self.xspeed = 0
            if self.ydest == TOP:
                if self.reached_dest(self.y, self.ydest, self.yspeed):
                    dy = self.ydest - self.y 
                    self.yspeed = 0
            else:
                yd = self.app.get_height(self.ydest, self.holding)
                if self.reached_dest(self.y, yd, self.yspeed):
                    dy = yd - self.y 
                    self.yspeed = 0
            self.move(dx, dy)
            if self.x == self.xdest and self.y == yd:
                if self.commands == []:
                    self.active = False
                    self.lifting = False
                else:
                    self.commands.pop(0)()

    def is_animating(self):
        return self.active or self.lifting

    def lift(self):
        print('lift {}'.format(self))
        self.active = True
        self.lifting = True
        self.yspeed = -self.speed
        self.xspeed = 0
        self.xdest = slot2pos(pos2slot(self.x))
        self.ydest = TOP

    def drop(self, y):
        print('drop {} {}'.format(y, self))
        self.active = True
        self.yspeed = self.speed
        self.xspeed = 0
        self.xdest = slot2pos(pos2slot(self.x))
        self.ydest = y


    def move(self, dx, dy):
        self.x += dx
        self.y += dy
        if self.holding is not None:
            self.holding.move(dx, dy)

    def move_to(self, s):
        print('move_to {} {}'.format(s, self))
        self.xdest = slot2pos(s)
        self.ydest = TOP
        self.yspeed = 0
        if self.x < self.xdest:
            self.xspeed = self.speed
        else:
            self.xspeed = -self.speed

    def at_slot(self, s):
        pos = slot2pos(s)
        return abs(self.x - pos) < 4

    def at_top(self):
        return abs(self.y - TOP < 4)

    def is_at(self, b):
        return abs(self.x - b.x) < 4 and abs(self.y - b.y) < 4


    def hold(self, slot):
        print('hold {}'.format(self))
        self.active = True
        self.flip_gripper()
        self.holding = self.app.slots[slot].pop()
        if self.app.slots[slot] == []:
            self.app.send_msg('[r_(holding({0})), f_(on_table({0}))]'.format(self.holding.i))
        else:
            on = str(self.app.slots[slot][-1].i)
            self.app.send_msg('[r_(holding({0})), f_(on({0}, {1}))]'.format(self.holding.i, on))
        

    def is_holding(self, bid):
        return self.holding is not None and self.holding.i == bid

    def release(self, slot):
        print('release {}'.format(self))
        self.active = True
        if self.app.slots[slot] == []:
            self.app.send_msg('[f_(holding({0})), r_(on_table({0}))]'.format(self.holding.i))
        else:
            on = str(self.app.slots[slot][-1].i)
            self.app.send_msg('[f_(holding({0})), r_(on({0}, {1}))]'.format(self.holding.i, on))
        self.app.slots[slot].append(self.holding)
        self.holding = None
        self.flip_gripper()

    def flip_gripper(self):
        self.closed = not self.closed

    def rectangles(self):
        yield (self.x + BLOCK_SIZE2 - ARM_SIZE2, 0, 
               self.x + BLOCK_SIZE2 - ARM_SIZE2 + ARM_SIZE, self.y)
        yield (self.x - ARM_SIZE, self.y - ARM_SIZE, 
               self.x - ARM_SIZE + BLOCK_SIZE + 2*ARM_SIZE, 
               self.y - ARM_SIZE + ARM_SIZE)

        if self.closed:
            offset = ARM_SIZE
        else:
            offset = 2*ARM_SIZE
        yield (self.x - offset, self.y - ARM_SIZE, 
               self.x - offset+ARM_SIZE, self.y - ARM_SIZE+BLOCK_SIZE)
        yield (self.x + BLOCK_SIZE + offset - ARM_SIZE, 
               self.y - ARM_SIZE, 
               self.x + BLOCK_SIZE + offset - ARM_SIZE+ARM_SIZE, 
               self.y - ARM_SIZE+BLOCK_SIZE)

class BlocksApp(object):

    def __init__(self, master):
        self._master = master
        master.title("TR - Single Arm Block World")
        master.protocol("WM_DELETE_WINDOW", self.closeEvent)
        frame1 = tk.Frame(master,bg='black')
        frame1.pack(fill=tk.BOTH, expand=1)
        master.geometry("{0}x{1}".format(WIDTH, HEIGHT))
        self.canvas = tk.Canvas(frame1, bg = "white")
        self.canvas.pack(fill=tk.BOTH, expand=1)
        self.x = 0
        self.y = 0
        self.slots = [[] for _ in range(NUM_SLOTS+1)]
        self.blocks = []
        for s, i, c in BLOCKS:
            b = Block(slot2pos(s), TABLE_POS-BLOCK_SIZE, i, c)
            self.blocks.append(b)
            self.slots[s].append(b)
        self.arms = [Arm(1, slot2pos(12), TOP, self)]
        self.client = pedroclient.PedroClient()
        self.client.register('oneArmSim')
        self.pedro = Pedro(self)
        self.pedro.start()
        self.animating = False
        self.interacting = False
        self.moving_block = None
        self.tasks = {}
        self.task_colours = {}
        self.table_colours = [0,0,0]
        self.current_action = None
        #self.qp = QtGui.QPainter()
        self.show()
        self.percepts_handle = None
        self.canvas.bind("<Button-1>", self.mousePressEvent)
        self.canvas.bind("<B1-Motion>", self.mouseMoveEvent)
        self.canvas.bind("<ButtonRelease-1>", self.mouseReleaseEvent)
        self.canvas.bind_all("<Key>", self.keyPressEvent)
        self._master.bind("<<pedro>>", self.pedro_message)
        self.task_actions = {}

    def closeEvent(self):
        self.client.p2p('messages:{0}'.format(self.agent), 'terminate_agent')
        time.sleep(1)
        self._master.destroy()

    def keyPressEvent(self, event):
        key = event.keysym
        if key == 'space':
            if self.interacting:
                self.interacting = False
                self.show()
                if self.percepts_handle :
                    self.send_state()
                self.client.p2p('messages:{0}'.format(self.agent),
                                'resume_agent')
                self.start_animate()
            else:
                self.interacting = True
                self.show()
                self.client.p2p('messages:{0}'.format(self.agent),
                                'suspend_agent')
                self.stop_animate()
        elif key == 'Left':
            self.slow_arms()
            self.show()
        elif key == 'Right':
            self.quick_arms()
            self.show()

    def slow_arms(self):
        for arm in self.arms:
            arm.slow()
            
    def quick_arms(self):
        for arm in self.arms:
            arm.quick()
            
            

    def mousePressEvent(self, event):
        if not self.interacting:
            return
        sid = pos2slot(event.x)
        s = self.slots[sid]
        self.moving_block = None
        if s is None or s == []:
            return
        pos =  pos2slotpos(event.y)
        if len(s) <= pos:
            return
        self.moving_block = s[pos]
        blocks = []
        shuffling = False
        for i, b in enumerate(self.slots[sid]):
            if i == pos:
                shuffling = True
            else:
                blocks.append(b)
                if shuffling:
                    b.move(0, BLOCK_SIZE)
        self.slots[sid] = blocks
        self.show()
        self.save_x = event.x
        self.save_y = event.y
        


    def mouseReleaseEvent(self, event):
        if not self.interacting:
            return
        if self.moving_block is None:
            return
        sid = pos2slot(event.x)
        if sid == 0:
            sid = 1
        if self.arms[0].at_slot(sid) and not self.arms[0].at_top() :
            if sid == 1:
                sid = 2
            else:
                sid -= 1
             
        pos =  pos2slotpos(event.y)
        if pos > len(self.slots[sid]):
            pos = len(self.slots[sid]) 
        blocks = []
        shuffling = False
        for i, b in enumerate(self.slots[sid]):
            if i == pos:
                self.moving_block.move_abs(slot2pos(sid), slotpos2pos(pos))
                blocks.append(self.moving_block)
                shuffling = True
            if shuffling:
                b.move(0, -BLOCK_SIZE)
            blocks.append(b)
        if not shuffling:
            self.moving_block.move_abs(slot2pos(sid), slotpos2pos(pos))
            blocks.append(self.moving_block)
        self.moving_block = None
        self.slots[sid] = blocks
        self.show()

    def mouseMoveEvent(self, event):
        if not self.interacting:
            return
        if self.moving_block is None:
            return
        x,y = event.x, event.y
        self.moving_block.move(x - self.save_x, y - self.save_y)
        self.show()
        self.save_x, self.save_y = x,y
 
    def send_state(self):
        messages = []
        if self.arms[0].holding:
            messages.append('r_(holding({0}))'.format(self.arms[0].holding.i))
        messages.append('fa_(on_table(_))')
        messages.append('fa_(on(_,_))')
            
        
        for i,s in enumerate(self.slots):
            if s is not None and s != []:
                for j,b in enumerate(s):
                    if j == 0:
                        messages.append('r_(on_table({0}))'.format(s[0].i))
                    else:
                        messages.append('r_(on({0}, {1}))'.format(s[j].i, s[j-1].i))


        self.send_msg('['+','.join(messages)+']')
        


        
    def pedro_message(self, _):
        self.process_message(message_queue.get())


    def set_resource_colours(self, lst):
        for a in self.arms:
            a.colour = 0
        if lst.get_type() == pedroclient.PObject.listtype:
            task_res = lst.toList()
            for tr in task_res:
                task = tr.args[0].val
                if task in  self.tasks:
                    arm = self.arms[0]
                    arm.colour = self.tasks[task][1]

    def set_waiting_info(self, lst):
        for t in self.tasks:
            self.tasks[t][2] = ''
        if lst.get_type() == pedroclient.PObject.listtype:
            task_res = lst.toList()
            for tr in task_res:
                task = tr.args[0].val
                res = str(tr.args[1])
                self.tasks[task][2] = res
 


    def process_message(self, pmsg):
        msg = pmsg
        msg = msg.args[2]
        #sender_task = pmsg.args[1].args[0].args[0].val
        if  msg.get_type() == pedroclient.PObject.atomtype and \
                msg.val == 'initialise_':
            self.percepts_handle = pmsg.args[1]
            self.agent = pmsg.args[1].args[0].args[1].val
            self.send_state()
            return
        if  msg.get_type() == pedroclient.PObject.atomtype and \
                msg.val == 'finish':
            return
        if msg.get_type() == pedroclient.PObject.structtype and \
           msg.functor.val ==  'locked_resources':
            self.set_resource_colours(msg.args[0])
            return
        elif msg.get_type() == pedroclient.PObject.structtype and \
             msg.functor.val ==  'waiting_resources':
            self.set_waiting_info(msg.args[0])
            return
        elif  msg.get_type() == pedroclient.PObject.structtype and \
                msg.functor.val ==  'task_data':
            used_labels = [self.tasks[k][1] for k in self.tasks]
            for i in range(1,10):
                if i not in used_labels:
                    break
            self.tasks[msg.args[0].val] = ['{0} : {1}'.format(msg.args[0].val,
                                                              str(msg.args[1])),
                                           i, '']
            self.show()
            return
        elif  msg.get_type() == pedroclient.PObject.structtype and \
                msg.functor.val ==  'remove_task':
            self.tasks.pop(msg.args[0].val)
            self.show()
            return
        
        elif msg.get_type() == pedroclient.PObject.structtype and \
             msg.functor.val == 'actions':
            sender_task = msg.args[0].val
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
  
    def process_stop_actions(self, stop_actions, sender_task):
        for a in stop_actions:
            print("stop_action", sender_task, str(a))
            cmd = a.functor.val
            cmd_args = a.args
            self.stop_arm()
        return

    def process_start_actions(self, start_actions, sender_task):
        if start_actions:
            self.start_animate()
        for a in start_actions:
            print("start_action", sender_task, str(a))
            cmd_args = a.args
            cmd = a.functor.val
            if cmd == 'pickup':
                self.current_action = ('pickup', cmd_args[0].val)
                slot = self.get_slot(cmd_args[0].val)
                self.pickup(slot)
            elif cmd == 'put_on_block':
                self.current_action = ('put_on_block', cmd_args[0].val)
                slot = self.get_slot(cmd_args[0].val)
                self.putdown(slot)
            elif cmd == 'put_on_table':
                slot = self.free_slot()
                self.current_action = ('put_on_table', slot)
                self.putdown(slot)
            else:
                assert (False), "COMMAND: {}".format(str(a))

               
    def get_height(self, sid, holding):
        slot = self.slots[sid]
        if len(slot) == 0:
            return TABLE_POS - BLOCK_SIZE
        elif holding:
            return slot[-1].y - BLOCK_SIZE
        else:
            return slot[-1].y

    def stop_arm(self):
        arm = self.arms[0]
        arm.set_commands([])
        self.current_action = None
        arm.stop()

    def get_slot(self, b):
        for i, s in enumerate(self.slots):
            if s is not None:
                for b1 in s:
                    if b1.i == b:
                        return i
        return None

    def free_slot(self):
        arm_slot = pos2slot(self.arms[0].x)
        d = 1
        while True:
            pos = arm_slot + d
            if pos in range(1,NUM_SLOTS+1) and self.slots[pos] == []:
                return pos
            if d > 0:
                d = -d
            else:
                d = 1-d

    def pickup(self, b_slot):
        arm = self.arms[0]
        #assert not arm.holding
        if arm.at_slot(b_slot):
            arm.set_commands([make_fun_1(arm.drop, b_slot), 
                              make_fun_1(arm.hold, b_slot),
                              make_fun_0(arm.lift)])
        elif not arm.at_top():
            arm.set_commands([make_fun_0(arm.lift),
                              make_fun_1(arm.move_to, b_slot), 
                              make_fun_1(arm.drop, b_slot), 
                              make_fun_1(arm.hold, b_slot),
                              make_fun_0(arm.lift)])
        else:
            arm.set_commands([make_fun_1(arm.move_to, b_slot), 
                              make_fun_1(arm.drop, b_slot), 
                              make_fun_1(arm.hold, b_slot),
                              make_fun_0(arm.lift)])

    def putdown(self, slot):
        arm = self.arms[0]
        if arm.holding:
            self.move_block(arm, slot)
            

    def move_block(self, arm, b2_slot):
        if arm.at_slot(b2_slot):
            arm.set_commands([make_fun_1(arm.drop, b2_slot), 
                              make_fun_1(arm.release, b2_slot),
                              make_fun_0(arm.lift)])
        elif not arm.at_top():
            arm.set_commands([make_fun_0(arm.lift),
                              make_fun_1(arm.move_to, b2_slot), 
                              make_fun_1(arm.drop, b2_slot), 
                              make_fun_1(arm.release, b2_slot),
                              make_fun_0(arm.lift)])
        else:
            arm.set_commands([make_fun_1(arm.move_to, b2_slot), 
                              make_fun_1(arm.drop, b2_slot), 
                              make_fun_1(arm.release, b2_slot),
                              make_fun_0(arm.lift)])
            
    def send_msg(self, msg):
        self.client.p2p(self.percepts_handle, msg)

    def show(self):
        self.canvas.delete(tk.ALL)
        self.draw()

    def draw_block(self, b):
        self.canvas.create_rectangle(b.x, b.y, b.x+BLOCK_SIZE, 
                                     b.y+BLOCK_SIZE,fill = b.c,
                                     outline=b.c)
        height = 6
        w = -4 + len(b.istr)
        self.canvas.create_text(b.x+(BLOCK_SIZE-w)/2, 
                                b.y+(BLOCK_SIZE + height/2)/2,
                                text = b.istr)
        
 
    def draw_blocks(self):
        for b in self.blocks:
            self.draw_block(b)

    def draw_tables(self):
        colour = TASK_COLOURS[self.table_colours[0]]
        self.canvas.create_rectangle(TABLE_START, TABLE_POS, 
                                     TABLE_START+TABLE_SIZE - 3*BLOCK_SIZE,
                                     TABLE_POS + TABLE_WIDTH,
                                     fill=colour, outline=colour)
        self.canvas.create_rectangle(TABLE_START+TABLE_SIZE/3 - BLOCK_SIZE, TABLE_POS, 
                                     TABLE_START+2*TABLE_SIZE/3 - 2*BLOCK_SIZE,
                                     TABLE_POS + TABLE_WIDTH,
                                     fill=colour, outline=colour)

    def draw_arm(self, arm):
        colour = TASK_COLOURS[arm.colour]
        for rect in arm.rectangles():
            self.canvas.create_rectangle(*rect, fill=colour, outline=colour)
        holding = arm.holding
        if holding is not None:
            self.draw_block(holding)

    def draw_arms(self):
        self.draw_arm(self.arms[0])

    def draw_tasks(self):
        pos = TABLE_START
        for t in self.tasks.values():
            tid = self.canvas.create_text(pos, 400, text=t[0], anchor=tk.W,
                                          fill = TASK_COLOURS[t[1]])
            bounds = self.canvas.bbox(tid) 
            pos += 100+bounds[2] - bounds[0]
                

    def draw(self):
        self.canvas.create_text(TABLE_SIZE/2, 20, 
                                text="Arm speed: {0}".format(self.arms[0].speed), fill="green")
        if self.interacting:
            self.canvas.create_text(TABLE_SIZE/2,50, 
                                    text = "Press the spacebar to continue",
                                    fill="red")
        else:
            self.canvas.create_text(TABLE_SIZE/2,50, 
                                    text = "Press the spacebar to suspend",
                                    fill="green")
        self.draw_blocks()
        self.draw_tables()
        self.draw_arms()
        self.draw_tasks()

    def start_animate(self):
        if not self.animating:
            if self.current_action is not None:
                if self.current_action[0] == 'pickup':
                    slot = self.get_slot(self.current_action[1])
                    self.pickup(slot)
                elif self.current_action[0] == 'put_on_block':
                    slot = self.get_slot(self.current_action[1])
                    self.putdown(slot)
                else:
                    slot = self.free_slot()
                    self.putdown(slot)
            self._master.after(10, self.animate)
            self.animating = True

    def stop_animate(self):
        if self.animating:
            self.animating = False

    def animate(self):
        if not self.animating:
            return
        if not self.arms[0].is_animating() :
            self.stop_animate()
            return
        self.arms[0].step()
        self.show()
        self._master.after(10, self.animate)
        

if __name__ == "__main__":
    root = tk.Tk()
    app = BlocksApp(root)
    root.mainloop()


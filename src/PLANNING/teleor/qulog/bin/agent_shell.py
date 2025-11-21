#!/usr/bin/env python3

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
import threading
import pedroclient
import argparse
from help_agent_shell import *

IS_VERSION_2 = sys.version_info < (3, 0)
if IS_VERSION_2:
    import Queue
    import Tkinter as tk
    import tkMessageBox
    import tkFileDialog
else:
    import queue as Queue
    import tkinter as tk
    from tkinter import messagebox as tkMessageBox
    from tkinter import filedialog as tkFileDialog



PARSER = pedroclient.PedroParser()

DELAY = 500

message_queue = Queue.Queue()

def functor_of(text):
    functor, _, _ = text.partition('(')
    return functor

def valid_handle(handle):
    """ Check if handle is a valid Pedro handle
    valid_handle(str) -> bool
    """
    try:
        addr = PARSER.parse(handle)
    except:
        return False
    if handle is None:
        return False

    if isinstance(addr, pedroclient.PStruct) and addr.functor.val == '@' \
       and addr.arity() == 2:
        host = addr.args[1]
        name = addr.args[0]
        if isinstance(name, pedroclient.PStruct):
            return False
        return isinstance(name, pedroclient.PAtom) and \
            isinstance(host, pedroclient.PAtom)
    return False

class MessageThread(threading.Thread):
    def __init__(self, parent):
        self.running = True
        self.parent = parent
        threading.Thread.__init__(self)
        self.daemon = True
        
    def run(self):
        while self.running:
            p2pmsg = self.parent.client.get_term()[0]
            # get the message
            message = p2pmsg.args[2]
            message_queue.put(str(message)+'\n')
            
    def stop(self):
        self.running = False

class HelpWindow(tk.Toplevel):
    def __init__(self):
        tk.Toplevel.__init__(self)
        # Size of Help Window
        self.geometry("470x400")
        self.text = tk.Text(self, wrap=tk.WORD)
        self.text.pack(fill=tk.BOTH, expand=1)
        tk.Button(self, text="Quit", command = self.quit).pack()
        self.text.insert(tk.CURRENT, HELP_TEXT)
        self.text.config(state='disabled')

    def quit(self):
        self.destroy()

class TRApp(object):
    def __init__(self, agent_name, robot_handle, host,
                 discreet_actions, master):
        master.title("Agent Shell")
        master.protocol("WM_DELETE_WINDOW", self.quit)
        self.master = master
        self.robot_handle = robot_handle
        #master.geometry("470x400")
        self.text = tk.Text(master)
        self.text.pack(fill=tk.BOTH, expand=1)
        action_frame = tk.Frame(master)
        action_frame.pack(fill=tk.X)
        tk.Label(action_frame, text="Actions: ").pack(side=tk.LEFT)
        self.entry = tk.Entry(action_frame)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=1)
        self.entry.bind("<Return>", self.send_return_actions)
        self.entry.focus()
        self.master.bind('<Up>', self.up_key)
        self.master.bind('<Down>', self.down_key)
        self.send = tk.Button(action_frame, text="Send", 
                              command=self.send_actions)
        self.send.pack(side=tk.LEFT)
        self.menubar = tk.Menu(master)
        master.config(menu=self.menubar)
        # File menu
        self.filemenu = tk.Menu(self.menubar)
        self.menubar.add_cascade(label="File", menu=self.filemenu)
        self.filemenu.add_command(label="Load History File",
                                  command=self.open_file)
        self.filemenu.add_command(label="Save History File",
                                  command=self.save_file)
        self.filemenu.add_command(label="Exit", command=self.quit)
        helpmenu = tk.Menu(self.menubar)
        self.menubar.add_cascade(label="Help", menu=helpmenu)
        helpmenu.add_command(label="Help", command=self.help)
        helpmenu.add_command(label="About", command=self.about)
        self.history = []
        self.discreet_actions = discreet_actions
        self.current_actions = []
        self.history_index = 0
        # connect to Pedro
        self.client = pedroclient.PedroClient(host)
        # register the process name with Pedro
        self.client.register(agent_name)
        # create the message handling thread
        self.message_thread = MessageThread(self)
        # and start it
        self.message_thread.start()
        self.text.tag_config("blue", foreground="blue")
        self.text.tag_config("red", foreground="red")
        self.history_loaded = False
        # send initialise
        if self.client.p2p(self.robot_handle, 'initialise_') == 0:
            self.text.insert(tk.END, 
                             "Illegal handle: {0}\n".format(self.robot_handle),
                             "red")
            
        self.text.insert(tk.END, 
                         "Shell ready - initialise sent\n",
                         "blue")
        self.text.see(tk.END)

    def open_file(self): 
        filename = tkFileDialog.askopenfilename()
        if filename:
            fd = open(filename, 'rU')
            self.history = [line.strip() for line in fd]
            fd.close()
            self.history_index = 0
            if self.history:
                self.set_entry_text()
                self.history_loaded = True

    def set_client(self, addr):
        self.tr_client_addr = addr
        
    def set_entry_text(self):
        self.entry.delete(0,tk.END)
        if self.history_index < len(self.history):
            self.entry.insert(0,self.history[self.history_index])

    def save_file(self):
        """Save the information in the hotel to a file."""
        filename = tkFileDialog.asksaveasfilename()
        if filename:
            fd = open(filename, 'w')
            fd.write('\n'.join(self.history))
            fd.close()

    def process_message(self):
        while not message_queue.empty():
            msg = message_queue.get()
            #msg = msg.replace("('$none_')", "()")
            self.text.insert(tk.END, "Percepts: " + msg)
        self.text.see(tk.END)
        self.master.after(DELAY,self.process_message)
 
    
    def quit(self):
        self.message_thread.stop()
        self.master.destroy()

    def up_key(self, _):
        if self.history_index > 0:
            self.history_index -= 1
        self.set_entry_text()

    def down_key(self, _):
        if self.history_index < len(self.history):
            self.history_index += 1
        self.set_entry_text()


    def send_return_actions(self, _):
        self.send_actions()

    def send_actions(self):
        text = self.entry.get()
        text = text.strip()
        #text = text.replace("()", "('$none_')")
        if self.history_index == len(self.history):
            self.history_loaded = False
        if self.history_loaded:
            self.history_index += 1
        else:
            self.history.append(text)
            self.history_index = len(self.history)
        self.set_entry_text()
        text = text.strip()
        if not text.startswith('['):
            text = '['+text
            if not text.endswith(']'):
                text = text+']'
        try:
            actions_term = PARSER.parse(text)
        except:
            self.text.insert(tk.END, 
                             "Illegal actions message: {0}\n".format(text),
                             "red")
            self.text.see(tk.END)
            return

            
        actions = text
        #paction = actions.replace("('$none_')", "()")
        self.text.insert(tk.END, "Actions: "+actions+'\n', "blue")
        text = text[1:-1]
        actions = [a.strip() for a in text.split(',')]
        commands_text = "actions(task, [" + ','.join(actions) + "])"
        self.text.insert(tk.END, "Controls: " + commands_text + "\n")
        # send a peer-to-peer message to the tr client
        if self.client.p2p(self.robot_handle, commands_text) == 0:
            self.text.insert(tk.END, 
                             "Illegal actions message: {0}\n".format(commands_text),
                             "red")
        self.text.see(tk.END)

    def about(self):
        tkMessageBox.showinfo(title="About Agent Shell", message=ABOUT_TEXT)

    def help(self):
        HelpWindow()


if __name__ == "__main__":
    parser = argparse.ArgumentParser() 
    parser.add_argument("agent_name", help="the process name of this shell")
    parser.add_argument("robot_handle", help="the handle of the robot the shell communicates with")
    parser.add_argument("-N", dest="pedro", default='localhost', 
                      help="the address of the Pedro server")
    parser.add_argument("discreet_actions", help="the discreet actions",
                        nargs='*')
    args = parser.parse_args()
    robot_handle = args.robot_handle
    if '@' not in robot_handle:
        robot_handle = robot_handle + '@localhost'
    else:
        parts = robot_handle.split('@')
        robot_handle = parts[0]
        for h in parts[1:]:
            robot_handle += "@'"+h+"'"
    #if ':' not in robot_handle:
    #    robot_handle = "'':"+robot_handle
    if valid_handle(robot_handle):
        root = tk.Tk()
        app = TRApp(args.agent_name, robot_handle, args.pedro,
                    args.discreet_actions, root)
        root.after(DELAY, app.process_message)
        root.mainloop()
    else:
        print("Invalid handle: {}".format(robot_handle))

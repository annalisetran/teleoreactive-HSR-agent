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
# pedroclient is the API for Pedro communication
import pedroclient
import threading
import argparse

import rospy
from std_msgs.msg import String


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



DELAY = 500

message_queue = Queue.Queue()


def controls_callback(data):
    print(data.data)
    message_queue.put(data.data)




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

class ROSApp(object):
    def __init__(self, wrapped, master):
        master.title("ROS Robot Shell")
        master.protocol("WM_DELETE_WINDOW", self.quit)
        self.master = master
        self.wrapped = wrapped
        #master.geometry("470x400")
        self.text = tk.Text(master)
        self.text.pack(fill=tk.BOTH, expand=1)
        percept_frame = tk.Frame(master)
        percept_frame.pack(fill=tk.X)
        tk.Label(percept_frame, text="Percepts: ").pack(side=tk.LEFT)
        self.entry = tk.Entry(percept_frame)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=1)
        self.entry.bind("<Return>", self.send_return_percepts)
        self.entry.focus()
        self.master.bind('<Up>', self.up_key)
        self.master.bind('<Down>', self.down_key)
        self.send = tk.Button(percept_frame, text="Send", 
                              command=self.send_percepts)
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
        self.history_index = 0

        
        self.text.tag_config("blue", foreground="blue")
        self.text.tag_config("red", foreground="red")
        self.parser = pedroclient.PedroParser()
        self.history_loaded = False
        
        # set up the ROS interface
        rospy.init_node('ros_robot_shell', anonymous=True)
        self.sub = rospy.Subscriber("controls", String, controls_callback)
        self.rate = rospy.Rate(10) 

        self.pub = rospy.Publisher('percepts', String, queue_size=10)
        
        self.text.insert(tk.END, 
                         "Shell ready\n",
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
            self.text.insert(tk.END, 'Actions received: ' + msg + '\n')
        self.text.see(tk.END)
        self.master.after(DELAY,self.process_message)
 
    
    def quit(self):
        self.master.destroy()

    def up_key(self, _):
        if self.history_index > 0:
            self.history_index -= 1
        self.set_entry_text()

    def down_key(self, _):
        if self.history_index < len(self.history):
            self.history_index += 1
        self.set_entry_text()


    def send_return_percepts(self, _):
        self.send_percepts()

    def send_percepts(self):
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
        if not text.startswith('['):
            text = '['+text
        if not text.endswith(']'):
            text = text+']'
            
        try:
            percept_term = self.parser.parse(text)
        except:
            self.text.insert(tk.END, 
                             "Illegal percepts message: {0}\n".format(text),
                             "red")
            self.text.see(tk.END)
            return
        if str(percept_term) != '[]' and \
           not isinstance(percept_term, pedroclient.PList):
            self.text.insert(tk.END, 
                             "Illegal percepts message: {0}\n".format(text),
                             "red")
            self.text.see(tk.END)
            return
        if not self.wrapped:
            percepts = text
        else:
            ps = []
            percept_list = []
            if isinstance(percept_term, pedroclient.PList):
                percept_list = percept_term.toList()
            for p in percept_list:
                pstr = str(p).strip()
                if not (pstr.startswith('r_(') or \
                        pstr.startswith('f_(') or \
                        pstr.startswith('u_(') or \
                        pstr.startswith('fa_(')) :
                    self.text.insert(tk.END, 
                                     "Illegal percepts message: {0}\n".\
                                     format(text), "red")
                    return
                else:
                    ps.append(pstr)
            percepts = '['+','.join(ps)+']'

        percepts = percepts+'\n'
        self.text.insert(tk.END,  'Percepts sent: ' + percepts, "blue")
        print("publish", percepts)
        self.pub.publish(percepts)
        self.text.see(tk.END)

    def about(self):
        tkMessageBox.showinfo(title="About TK Shell", message=ABOUT_TEXT)

    def help(self):
        HelpWindow()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-w", "--wrapped", action='store_true', 
                        help="for handle_percepts where the percepts are wrapped with r_ and f_")
    args = parser.parse_args()
    root = tk.Tk()
    app = ROSApp(args.wrapped, root)
    root.after(DELAY, app.process_message)
    root.mainloop()

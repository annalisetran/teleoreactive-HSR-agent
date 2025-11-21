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
from help_robot_shell import *

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



# In this case we want to respond in some way immediately a message arrives
# We do this below using a thread. Depending on the application this might
# instead create an event.
# The other approach (see Pedro documentation on Python API) is to set up
# the connection for async comms and then use notification_ready() to
# see if there are message to process 
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
            if str(message) == 'initialise_':
                # get the sender address
                percepts_addr = p2pmsg.args[1]
                self.parent.set_client(percepts_addr)
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
    def __init__(self, shell_name, host, wrapped, master):
        master.title("Robot Shell")
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
        # connect to Pedro
        self.client = pedroclient.PedroClient(host)
        # register the process name with Pedro
        self.client.register(shell_name)
        self.tr_client_addr = None
        # create the message handling thread
        self.message_thread = MessageThread(self)
        # and start it
        self.message_thread.start()
        self.text.tag_config("blue", foreground="blue")
        self.text.tag_config("red", foreground="red")
        self.parser = pedroclient.PedroParser()
        self.history_loaded = False
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
            self.text.insert(tk.END, msg)
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


    def send_return_percepts(self, _):
        self.send_percepts()

    def send_percepts(self):
        if self.tr_client_addr is None:
            self.text.insert(tk.END, 
                             "No agent connected\n",
                             "red")
            self.text.see(tk.END)
            return
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

        #rpercepts = percepts.replace("('$none_')", "()")
        self.text.insert(tk.END, percepts+'\n', "blue")
        # send a peer-to-peer message to the tr client
        if self.client.p2p(self.tr_client_addr, percepts) == 0:
            self.text.insert(tk.END, 
                             "Illegal percepts message: {0}\n".format(percepts),
                             "red")
        self.text.see(tk.END)

    def about(self):
        tkMessageBox.showinfo(title="About TK Shell", message=ABOUT_TEXT)

    def help(self):
        HelpWindow()


if __name__ == "__main__":
    parser = argparse.ArgumentParser() 
    parser.add_argument("shell_name", help="the process name of this shell")
    parser.add_argument("-N", dest="pedro", default='localhost', 
                      help="the address of the Pedro server")
    parser.add_argument("-w", "--wrapped", action='store_true', 
                        help="for handle_percepts where the percepts are wrapped with r_ and f_")
    args = parser.parse_args()
    root = tk.Tk()
    app = TRApp(args.shell_name, args.pedro, args.wrapped, root)
    root.after(DELAY, app.process_message)
    root.mainloop()

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
import pedroclient
import threading
import argparse

IS_VERSION_2 = sys.version_info < (3, 0)
if IS_VERSION_2:
    import Queue
    import Tkinter as tk
else:
    import queue as Queue
    import tkinter as tk
    
DEFAULT_ENTRIES = "5"
SEARCH_TEXT = "Received new percepts"
DELAY = 500

message_queue = Queue.Queue()

class MessageThread(threading.Thread):
    def __init__(self, client):
        self.running = True
        self.client = client
        threading.Thread.__init__(self)
        self.daemon = True
        
    def run(self):
        while self.running:
            
            p2pmsg = self.client.get_term()[0]
            #print str(p2pmsg)
            message = str(p2pmsg.args[2])[1:-1]
            message = message.replace('\\\\"', '"')
            if message.startswith('\nUser :::'):
                message_queue.put(message)
            elif message.startswith('\nTask evaluator'):
                message_queue.put(message)
            else:
                tr_addr = p2pmsg.args[1].args[0].args[1].val
                message_queue.put('\n'+str(tr_addr) +'::: '+message+'\n')
            
    def stop(self):
        self.running = False




class LoggerApp(object):
    def __init__(self, logger_name, host, master):
        master.title("Logger : " + logger_name)
        master.protocol("WM_DELETE_WINDOW", self.quit)
        self.master = master
        master.geometry("600x500")
        text_frame = tk.Frame(master)
        text_frame.pack(fill=tk.BOTH, expand=1)
        scrollbar = tk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.text = tk.Text(text_frame, yscrollcommand=scrollbar.set)
        self.text.pack(fill=tk.BOTH, expand=1)
        self.text.tag_config('orange', background='dark orange')
        scrollbar.config(command=self.text.yview)
        self.master.bind('<<msg>>', self.process_message)
        search_frame = tk.Frame(master)
        search_frame.pack()
        search_label = tk.Label(search_frame, text="Search: ")
        search_label.pack(side=tk.LEFT)
        self.search_box = tk.Entry(search_frame)
        self.search_box.pack(side=tk.LEFT)
        self.search_box.bind('<Return>', self.searchback)
        searchup = tk.Button(search_frame, text="Search up", 
                               command=self.search_up)
        searchup.pack(side=tk.LEFT)
        searchdown = tk.Button(search_frame, text="Search down", 
                               command=self.search_down)
        searchdown.pack(side=tk.LEFT)
        goend = tk.Button(search_frame, text="Go to End", 
                          command=self.goto_end)
        goend.pack(side=tk.LEFT)
        clear = tk.Button(search_frame, text="Clear", command=self.clear)
        clear.pack(side=tk.LEFT)
        snapshot_frame = tk.Frame(master)
        snapshot_frame.pack()
        snapshot_label = tk.Label(snapshot_frame,text="Number of Entries: ")
        snapshot_label.pack(side=tk.LEFT)
        var = tk.StringVar(root)
        var.set(DEFAULT_ENTRIES)
        self.snapshot_scale = \
            tk.Spinbox(snapshot_frame, from_=1,to=20,width = 5, 
                       textvariable=var)
        self.snapshot_scale.pack(side=tk.LEFT)
        snapshot_button = tk.Button(snapshot_frame, text="Snap", 
                                    command=self.snap)
        snapshot_button.pack(side=tk.LEFT)
        self.client = pedroclient.PedroClient(host)
        if self.client.register(logger_name):
            self.text.insert(tk.END, "Logger ready\n\n", "blue")
            self.text.see(tk.END)
        else:
            self.text.insert(tk.END, "Cannot register "+ \
                             logger_name + " with Pedro\n\n", "red")
            self.text.see(tk.END)
        self.message_thread = MessageThread(self.client)
        self.message_thread.start()
        self.text.tag_config("user_col", foreground="blue")
        self.text.tag_config("err_col", foreground="red")
        self.text.tag_config("eval_col", foreground="purple1")
        self.text.tag_config("per_col", foreground="cyan4")
        self.text.tag_config("msg_col", foreground="chocolate3")
        self.do_process = True
        self.search_pos = tk.END

    def clear(self):
        self.text.delete("1.0",tk.END)

    def goto_end(self):
        self.text.see(tk.END)
        self.text.mark_set("insert",tk.END)
        self.search_pos = tk.END
        self.text.tag_remove('orange','1.0',tk.END)

    def search_down(self):
        next_ = "{}+1c".format(self.search_pos)
        pos = self.text.search(self.search_box.get(), next_)
        self.text.tag_remove('orange','1.0',tk.END)
        if pos:
            self.search_pos = pos
            self.text.see(pos)
            self.text.mark_set("insert", pos)
            self.text.tag_add('orange', pos, 
                              "{}+{}c".format(pos, len(self.search_box.get())))
    def search_up(self):
        next_ = "{}+1c".format(self.search_pos)
        pos = self.text.search(self.search_box.get(), self.search_pos, 
                               backwards=True)
        self.text.tag_remove('orange','1.0',tk.END)
        if pos:
            self.search_pos = pos
            self.text.see(pos)
            self.text.mark_set("insert", pos)
            self.text.tag_add('orange', pos, 
                              "{}+{}c".format(pos, len(self.search_box.get())))
    def searchback(self, e):
        pos = self.text.search(self.search_box.get(), self.search_pos, 
                               backwards=True)
        self.text.tag_remove('orange','1.0',tk.END)
        if pos:
            self.search_pos = pos
            self.text.see(pos)
            self.text.mark_set("insert", pos)
            self.text.tag_add('orange', pos, 
                              "{}+{}c".format(pos, len(self.search_box.get())))
        
    def process_message(self):
        progress = False
        while self.do_process and not message_queue.empty():
            progress = True
            msg = message_queue.get()
            if "error(" in msg:
                self.text.insert(tk.END, msg, "err_col")
            elif "User :::" in msg:
                self.text.insert(tk.END, msg, "user_col")
            elif "Received new percept" in msg:
                self.text.insert(tk.END, msg, "per_col")
            elif "Received message" in msg:
                self.text.insert(tk.END, msg, "msg_col")
            elif "Current Percepts" in msg:
                self.text.insert(tk.END, msg, "eval_col")
            elif "Task evaluator " in msg:
                self.text.insert(tk.END, msg, "eval_col")
            else:
                self.text.insert(tk.END, msg)
        if progress: 
            self.text.see(tk.END)
        self.master.after(DELAY,self.process_message)
        
    def snap(self):
        self.do_process = False
        current_end = tk.END
        try:
            num = int(self.snapshot_scale.get())
        except:
            num = int(DEFAULT_ENTRIES)
        pos = current_end
        for i in range(num):
            pos = self.text.search(SEARCH_TEXT, pos, backwards=True)
            if pos == "":
                pos = "1.0"
                break
        SnapWindow(self.text.get(pos, current_end))
        self.do_process = True
 
    def quit(self):
        self.message_thread.stop()
        self.master.destroy()

class SnapWindow(tk.Toplevel):
    def __init__(self, text) :
        tk.Toplevel.__init__(self)
        self.title("Snapshot")
        #self.geometry("600x450")
        self.text = tk.Text(self)
        self.text.pack(fill=tk.BOTH, expand=1)
        self.text.insert(tk.END, text)
        self.text.see(tk.END)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("logger_name", help="the process name of this logger")
    parser.add_argument("-N", dest="pedro", default='localhost', 
                      help="the address of the Pedro server")
    args = parser.parse_args()
    root = tk.Tk()
    app = LoggerApp(args.logger_name, args.pedro, root)
    root.after(DELAY, app.process_message)
    root.mainloop()



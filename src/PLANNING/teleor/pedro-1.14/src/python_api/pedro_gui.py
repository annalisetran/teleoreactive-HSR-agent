#!/usr/bin/env python

#  Copyright (C) 2006, 2007, 2008 Peter Robinson
#  Email: pjr@itee.uq.edu.au
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

""" This is a simple GUI that connects to the Pedro server and can be
used to manage subscriptions and send notifications.
The top window displays received notifications and P2P messages.
The second window displays a list of current subscriptions.
A Right-Click in this window will pop up a menu for managing subscriptions
and name registration.
A Left-Click will choose a subscrription.
"""

import sys

IS_VERSION_2 = sys.version_info < (3, 0)
if IS_VERSION_2:
    import Queue
    import Tkinter as tk
    import tkMessageBox

else:
    import queue as Queue
    import tkinter as tk
    from tkinter import messagebox as tkMessageBox
    

from pedroclient import *
import threading

port = 4550
machine = 'localhost'

help_text = "Use the Commands menu for the following choices:\n\n\
* Add Subscription - add a subscription\n\
* Remove Subscription - remove highlighed subscription\n\
* Register - register a name with the Pedro server\n\
* Deregister - deregister name\n\
* Help - this message\n\n\
Notifications received are displayed in top window\n\n\
Notifications can be entered in the Notification: text entry\n\n\
Peer-to-peer messages can be sent using the bottom two text entries:\n\n\
* The P2P address has the form name@machine where name is a registered name \
and machine is the machine on which that process is running.\n\
* The Message is any Prolog term\n\n\
See the Pedro manual for examples of subscriptions and Prolog terms."

about_text = """A simple GUI for Pedro communication.

Copyright 2011 Peter Robinson

email: pjr@itee.uq.edu.au
"""

message_queue = Queue.Queue()

class Pedro(threading.Thread):
    def __init__(self, app):
        self.running = True
        self.app = app
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        while (self.running):
            msg = self.app.client.get_notification()[0]
            message_queue.put(msg)
            self.app._master.event_generate("<<pedro>>")

    def stop(self):
        self.running = False


class Help(tk.Toplevel):
    def __init__(self, master):
        tk.Toplevel.__init__(self, master)
        self.title = "Help"
        self.transient(master)
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.text = tk.Text(self, width = 90, height = 25)
        self.text.pack()
        self.text.insert(tk.END, help_text)
        
    def cancel(self):
        self.destroy()
        
class About(tk.Toplevel):
    def __init__(self, master):
        tk.Toplevel.__init__(self, master)
        self.title = "About"
        self.transient(master)
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.text = tk.Text(self, width = 40, height = 10)
        self.text.pack()
        self.text.insert(tk.END, about_text)
        
    def cancel(self):
        self.destroy()

class Subscribe(tk.Toplevel):
    def __init__(self, master, parent):
        tk.Toplevel.__init__(self, master)
        self._parent = parent
        self.title = "Subscribe"
        self.transient(master)
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.frame1 = tk.Frame(self)
        self.frame1.pack(fill=tk.X)
        self.frame2 = tk.Frame(self)
        self.frame2.pack(fill=tk.X)
        self.frame3 = tk.Frame(self)
        self.frame3.pack()
        tk.Label(self.frame1,text="Head:").pack(side=tk.LEFT,expand=1, anchor=tk.W)
        self.head_entry = tk.Entry(self.frame1, width=40)
        self.head_entry.pack(side=tk.LEFT,expand=1)
        tk.Label(self.frame2,text="Body:").pack(side=tk.LEFT,expand=1, anchor=tk.W)
        self.body_entry = tk.Entry(self.frame2, width=40)
        self.body_entry.pack(side=tk.LEFT,expand=1)
        cancel_button =tk.Button(self.frame3, text="Cancel", command = self.cancel)
        cancel_button.pack(side=tk.LEFT)
        ok_button = tk.Button(self.frame3, text="OK", command = self.ok)
        ok_button.pack(side=tk.LEFT)
        self.grab_set()
        self.head_entry.focus_set()

    def ok(self):
        head = self.head_entry.get()
        body = self.body_entry.get()
        self._parent.add_sub(head, body)
        self.destroy()
        
    def cancel(self):
        self.destroy()
        
class Unsubscribe(tk.Toplevel):
    def __init__(self, master, parent):
        tk.Toplevel.__init__(self, master)
        self._parent = parent
        self.title = "Unsubscribe"
        self.transient(master)
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.frame1 = tk.Frame(self)
        self.frame1.pack(fill=tk.X)
        tk.Label(self.frame1,text="ID:").pack(side=tk.LEFT,expand=1, anchor=tk.W)
        self.id_entry = tk.Entry(self.frame1, width=10)
        self.id_entry.pack(side=tk.LEFT)
        self.frame3 = tk.Frame(self)
        self.frame3.pack()
        cancel_button =tk.Button(self.frame3, text="Cancel", command = self.cancel)
        cancel_button.pack(side=tk.LEFT)
        ok_button = tk.Button(self.frame3, text="OK", command = self.ok)
        ok_button.pack(side=tk.LEFT)
        self.grab_set()
        self.id_entry.focus_set()

    def ok(self):
        id = int(self.id_entry.get())
        self._parent.remove_sub(id)
        self.destroy()
        
    def cancel(self):
        self.destroy()

class Register(tk.Toplevel):
    def __init__(self, master, parent):
        tk.Toplevel.__init__(self, master)
        self._parent = parent
        self.title = "Register"
        self.transient(master)
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.frame1 = tk.Frame(self)
        self.frame1.pack(fill=tk.X)
        tk.Label(self.frame1,text="Name:").pack(side=tk.LEFT,expand=1, anchor=tk.W)
        self.name_entry = tk.Entry(self.frame1, width=10)
        self.name_entry.pack(side=tk.LEFT)
        self.frame3 = tk.Frame(self)
        self.frame3.pack()
        cancel_button =tk.Button(self.frame3, text="Cancel", command = self.cancel)
        cancel_button.pack(side=tk.LEFT)
        ok_button = tk.Button(self.frame3, text="OK", command = self.ok)
        ok_button.pack(side=tk.LEFT)
        self.grab_set()
        self.name_entry.focus_set()

    def ok(self):
        name = self.name_entry.get()
        self._parent.register_client(name)
        self.destroy()
        
    def cancel(self):
        self.destroy()

        
class GUIApp(object):

    def __init__(self, master):
        self._master = master
        master.title("Pedro Client")
        master.protocol("WM_DELETE_WINDOW", self.closeEvent)
        self.client = PedroClient(machine, port)
        self.pedro = Pedro(self)
        self.pedro.start()
 
        self.menubar = tk.Menu(master)
        master.config(menu=self.menubar)
        # Command menu
        self.commandmenu = tk.Menu(self.menubar)
        self.menubar.add_cascade(label="Commands", menu=self.commandmenu)
        self.commandmenu.add_command(label="Add Subscription",
                                     command=self.add_subscription)
        self.commandmenu.add_command(label="Remove Subscription",
                                     command=self.remove_subscription)
        self.commandmenu.add_command(label="Register",
                                     command=self.register)
        self.commandmenu.add_command(label="Deregister",
                                     command=self.deregister)

        self.helpmenu = tk.Menu(self.menubar)
        self.menubar.add_cascade(label="Help", menu=self.helpmenu)
        self.helpmenu.add_command(label="Help",
                                     command=self.help)
        self.helpmenu.add_command(label="About",
                                     command=self.about)

        self.message_text = tk.Text(master, width = 100, height = 15)
        self.message_text.pack()
        self.notification_text = tk.Text(master, width = 100, height = 10)
        self.notification_text.pack()
        self.commands_frame = tk.Frame(master)
        self.commands_frame.pack(fill=tk.X)
        self.commands_frame1 = tk.Frame(self.commands_frame)
        self.commands_frame1.pack(fill=tk.X)
        self.commands_frame2 = tk.Frame(self.commands_frame)
        self.commands_frame2.pack(fill=tk.X)
        tk.Label(self.commands_frame1,text="Notification:").pack(side=tk.LEFT,expand=1, anchor=tk.W)
        self.notification_entry = tk.Entry(self.commands_frame1, width=80)
        self.notification_entry.pack(side=tk.LEFT,expand=1)
        tk.Button(self.commands_frame1,text="Send",command=self.send_notification).pack(side=tk.LEFT,expand=1, anchor=tk.E)

        tk.Label(self.commands_frame2,text="P2PAddress:").pack(side=tk.LEFT,expand=1, anchor=tk.W)
        self.p2p_addr_entry = tk.Entry(self.commands_frame2, width=30)
        self.p2p_addr_entry.pack(side=tk.LEFT,expand=1)
        tk.Label(self.commands_frame2,text="Message:").pack(side=tk.LEFT,expand=1)        
        self.p2p_msg_entry = tk.Entry(self.commands_frame2, width=30)
        self.p2p_msg_entry.pack(side=tk.LEFT,expand=1)
        
        tk.Button(self.commands_frame2,text="Send",command=self.send_p2p).pack(side=tk.LEFT,expand=1, anchor=tk.E)
        self._master.bind("<<pedro>>", self.pedro_message)
        self.subs = {}
        
    def closeEvent(self):
        self._master.destroy()

    def add_subscription(self):
        Subscribe(self._master, self)

    def add_sub(self, head, body):
        id = self.client.subscribe(head, body)
        if id == 0:
            tkMessageBox.showerror(
                "Error",
                "Invalid subscription"
                )
        else:
           self.subs[id] = (head, body)
           self.notification_text.insert(tk.END, "{}\t{}\t{}\n".format(id, head, body))

    def remove_subscription(self):
        Unsubscribe(self._master, self)

    def remove_sub(self, subid):
        id = self.client.unsubscribe(subid)
        if id == 0:
            tkMessageBox.showerror(
                "Error",
                "Cannot remove subscription"
            )
        else:
            self.subs.pop(id)
            self.notification_text.delete("1.0",tk.END)
            keys = sorted(self.subs.keys())
            data = [(k, self.subs[k][0],
                     self.subs[k][1]) for k in keys]
            text_list = ["{}\t{}\t{}\n".format(k, head, body) for (k, head, body) in data]
            text = ''.join(text_list)
            self.notification_text.insert(tk.END, text)

    def register(self):
        Register(self._master, self)

    def register_client(self, name):
        id = self.client.register(name)
        if id == 0:
            tkMessageBox.showerror(
                "Error",
                "Cannot register"
            )
        else:
            self._master.title("Pedro Client: {}".format(name))
            
    def deregister(self):
        id = self.client.deregister()
        if id == 0:
            tkMessageBox.showerror(
                "Error",
                "Cannot deregister"
            )
        else:
            self._master.title("Pedro Client")
            
    def help(self):
        Help(self._master)

    def about(self):
        About(self._master)
        
    def send_notification(self):
        id = self.client.notify(self.notification_entry.get())
        if id == 0:
            tkMessageBox.showerror(
                "Error",
                "Cannot send notification"
            )
 
    def send_p2p(self):
        id = self.client.p2p(self.p2p_addr_entry.get(),
                             self.p2p_msg_entry.get())
        
        if id == 0:
            tkMessageBox.showerror(
                "Error",
                "Cannot send P2P message"
            )
   
    def pedro_message(self, _):
        text = message_queue.get()
        self.message_text.insert(tk.END, text + '\n')
        
if __name__ == "__main__":
    argc = len(sys.argv)
    if argc > 1:
        i = 1
        while i < argc:
            if sys.argv[i] == '-P':
                i = i+1
                port = int(sys.argv[i])
            elif sys.argv[i] == '-M':
                i = i+1
                machine = sys.argv[i]
            else:
                print("Usage: pedro_gui.py [-P port] [-M machine]")
                print("")
                print(help_text)
                sys.exit()
            i = i+1

    root = tk.Tk()
    app = GUIApp(root)
    root.mainloop()


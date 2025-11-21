# Copyright 2018 Keith Clark, Peter Robinson
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

import tkinter as tk

class SearchDialog(tk.Toplevel):
    def __init__(self, master, parent):
        tk.Toplevel.__init__(self, master)
        self.parent = parent
        self.title("Search")
        frame1 = tk.Frame(self)
        frame1.pack(expand=1, fill=tk.X)
        frame2 = tk.Frame(self)
        frame2.pack(expand=1, fill=tk.X)
        find_label = tk.Label(frame1, text= "Find:   ")
        find_label.pack(side=tk.LEFT)
        self.entry = tk.Entry(frame1, width=50)
        self.entry.pack(side=tk.LEFT)
        find_button = tk.Button(frame1, text="Find Next", command=self.find)
        find_button.pack(side=tk.LEFT)
        dir_label = tk.Label(frame2, text= "Direction:")
        dir_label.pack(side=tk.LEFT)
        self.var = tk.IntVar()
        self.var.set(1)
        r1 = tk.Radiobutton(frame2, text="Forward", variable=self.var, value=1)
        r1.pack(side=tk.LEFT, anchor=tk.W )
        r2 = tk.Radiobutton(frame2, text="Backwards", variable=self.var,
                            value=2)
        r2.pack(side=tk.LEFT, anchor=tk.W )

    def find(self):
        self.parent.find(self.entry.get(), (self.var.get() == 1))


class SearchReplaceDialog(tk.Toplevel):
    def __init__(self, master, parent):
        tk.Toplevel.__init__(self, master)
        self.parent = parent
        self.title("Search and Replace")
        left_frame = tk.Frame(self)
        left_frame.pack(side=tk.LEFT,expand=1, fill=tk.X)
        right_frame = tk.Frame(self)
        right_frame.pack(side=tk.LEFT,expand=1, fill=tk.X)
        frame1 = tk.Frame(left_frame)
        frame1.pack(expand=1, fill=tk.X)
        frame2 = tk.Frame(left_frame)
        frame2.pack(expand=1, fill=tk.X)
        frame3 = tk.Frame(left_frame)
        frame3.pack(expand=1, fill=tk.X)
        find_label = tk.Label(frame1, text= "Find:")
        find_label.pack(side=tk.LEFT)
        self.entry = tk.Entry(frame1, width=50)
        self.entry.pack(side=tk.RIGHT)
        
        replace_label = tk.Label(frame2, text= "Replace:")
        replace_label.pack(side=tk.LEFT)
        self.replace_entry = tk.Entry(frame2, width=50)
        self.replace_entry.pack(side=tk.LEFT)
        
        dir_label = tk.Label(frame3, text= "Direction:")
        dir_label.pack(side=tk.LEFT)
        self.var = tk.IntVar()
        self.var.set(1)
        r1 = tk.Radiobutton(frame3, text="Forward", variable=self.var, value=1)
        r1.pack(side=tk.LEFT, anchor=tk.W )
        r2 = tk.Radiobutton(frame3, text="Backwards", variable=self.var,
                            value=2)
        r2.pack(side=tk.LEFT, anchor=tk.W )

        find_button = tk.Button(right_frame, text="Find", command=self.find)
        find_button.pack(anchor=tk.W)
        replace_button = tk.Button(right_frame, text="Replace",
                                   command=self.replace)
        replace_button.pack(anchor=tk.W)
        find_replace_button = tk.Button(right_frame, text="Find+Replace",
                                        command=self.find_replace)
        find_replace_button.pack(anchor=tk.W)

    def find(self):
        self.parent.find(self.entry.get(), (self.var.get() == 1), True)
        
    def replace(self):
        self.parent.search_replace(self.replace_entry.get())
        
    def find_replace(self):
        self.parent.find(self.entry.get(), (self.var.get() == 1), True)
        self.parent.search_replace(self.replace_entry.get())

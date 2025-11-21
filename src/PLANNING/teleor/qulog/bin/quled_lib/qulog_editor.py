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

import sys
import os

import tkinter as tk
from tkinter import messagebox, filedialog

from quled_lib import editwin
from quled_lib import search_dialog

        
class EditorApp(object):

    def __init__(self, master, filename):
        self._master = master
        self.file_path = None
        self.set_title()
        master.protocol("WM_DELETE_WINDOW", self.file_quit)
        
        self.scrollbar = tk.Scrollbar(master)
        self.scrollbar.pack(side = tk.RIGHT, fill=tk.Y)
        self.editor = editwin.Editwin(master, self, width = 80, height = 45,
                              undo=True, autoseparators=True, maxundo=-1,
                              yscrollcommand = self.scrollbar.set)
        
        self.menubar = tk.Menu(master)
        master.config(menu=self.menubar)
        filemenu = tk.Menu(self.menubar)
        filemenu.add_command(label="New", command=self.file_new,
                             accelerator="Ctrl+N")
        filemenu.add_command(label="Open...", command=self.file_open,
                             accelerator="Ctrl+O")
        filemenu.add_command(label="Save", command=self.file_save,
                             accelerator="Ctrl+S")
        filemenu.add_command(label="Save As...", command=self.file_save_as,
                             accelerator="Ctrl+W")
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.file_quit,
                             accelerator="Ctrl+X Ctrl+C")
        self.menubar.add_cascade(label="File", underline=0, menu=filemenu)

        editmenu = tk.Menu(self.menubar)
        editmenu.add_command(label="Undo", command=self.editor.undo,
                             accelerator="Ctrl+Z")
        editmenu.add_command(label="Redo", command=self.editor.redo,
                             accelerator="Ctrl+Shift+Z")
        editmenu.add_separator()
        editmenu.add_command(label="Cut", \
                             accelerator="Ctrl+X", \
                             command= self.editor.cut)
        editmenu.add_command(label="Copy", \
                             accelerator="Ctrl+C", \
                             command=self.editor.copy)
        editmenu.add_command(label="Paste", \
                             accelerator="Ctrl+V", \
                             command=self.editor.paste)
        editmenu.add_separator()
        editmenu.add_command(label="Search", \
                             command=self.search)
        self.menubar.add_cascade(label="Edit", underline=0, menu=editmenu)
        editmenu.add_command(label="Replace", \
                             command=self.replace)

        #editmenu.add_separator()
        qulogmenu = tk.Menu(self.menubar)
        qulogmenu.add_command(label="Comment Region",
                             command=self.editor.comment_region)
        qulogmenu.add_command(label="Uncomment Region",
                             command=self.editor.uncomment_region)
        qulogmenu.add_separator()
        qulogmenu.add_command(label="Indent All",
                              command=self.editor.indent_all)
        self.menubar.add_cascade(label="Qulog", underline=0, menu=qulogmenu)


        self.editor.pack(fill=tk.BOTH, expand=1)
        self.scrollbar.config(command = self.editor.yview )
        status_frame = tk.Frame(master)
        status_frame.pack(fill=tk.X)
        self.status_label = tk.Label(status_frame)
        self.status_label.pack(side=tk.LEFT, anchor=tk.E)
        self.rc_label = tk.Label(status_frame)
        self.rc_label.pack(side=tk.RIGHT, anchor=tk.W)
        
        self.cwd = None
        if filename:
            self.cwd = os.getcwd()
            self.file_open(None, os.path.join(self.cwd, filename))


    def search(self):
        search_dialog.SearchDialog(self._master, self)
        return "break"
    def replace(self):
        search_dialog.SearchReplaceDialog(self._master, self)
        return "break"

    def find(self, search, forward, replace=False):
        self.editor.find(search, forward, replace)
    def search_replace(self, replace):
        self.editor.search_replace(replace)
        
    def file_new(self, event=None):
        result = self.save_if_modified()
        if result != None: 
            self.editor.clear()
            self.editor.edit_reset()
            self.set_title()
        return "break"

    def save_if_modified(self, event=None):
        if self.editor.edit_modified(): 
            response = messagebox.askyesnocancel("Save?", "This document has been modified. Do you want to save changes?") 
            if response: 
                result = self.save_the_file()
                if result == "saved": 
                    return True
                else: 
                    return None
            else:
                return response 
        else: 
            return True

    def file_open(self, event=None, filepath=None):
        result = self.save_if_modified()
        if result != None: 
            if filepath == None:
                filepath = filedialog.askopenfilename(filetypes=(('Qulog/Teleor', '*.qlg'),))
            if filepath:
                with open(filepath, encoding="utf-8") as f:
                    fileContents = f.read()
                    self.editor.load_text(fileContents)
                    self.file_path = filepath
                    self.editor.edit_reset()
                    self.set_title()
        return "break"
                    
    def file_save(self, event=None):
        self.save_the_file()
        return "break"

    def save_the_file(self):    
        if self.file_path == None:
            result = self.file_save_as()
        else:
            result = self.file_save_as(filepath=self.file_path)
        return result

    def file_save_as(self, event=None, filepath=None):
        if filepath == None:
            filepath = filedialog.asksaveasfilename(filetypes=(('Qulog/Teleor', '*.qlg'),))
        if filepath:
            try:
                with open(filepath, 'wb') as f:
                    text = self.editor.save_text()
                    f.write(bytes(text, 'UTF-8'))
                    self.file_path = filepath
                    self.set_title()
                    return "saved"
            except FileNotFoundError:
                response = messagebox.error("Save", "Cannot save to "+filepath)
                return "cancelled"

    def file_quit(self, event=None):
        result = self.save_if_modified()
        if result != None: 
            self._master.destroy() 

    def set_title(self, event=None):
        if self.file_path != None:
            title = os.path.basename(self.file_path)
        else:
            title = "Untitled"
        self._master.title(title + " - Qulog Editor")


def main(filename):
    root = tk.Tk()
    app = EditorApp(root, filename)
    root.mainloop()


if __name__ == "__main__":
    argc = len(sys.argv)
    if argc > 1:
        filename = sys.argv[1]
    else:
        filename = None

    main(filename)

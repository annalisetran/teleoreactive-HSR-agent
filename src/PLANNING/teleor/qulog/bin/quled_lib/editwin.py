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

import re
import tkinter as tk

face_tags = {"stringface" : "#006400",
             "qatomface" : "#6b8e23",
             "commentface" : "#b83231",
             "keywordface" : "#9400d3",
             "functorface" : "#0000a0",
             "variableface" : "#855000",
             "constantface" : "#316994"}

keywords = ["once", "or_while", "commit_while", "min_time", 
            "forall", "not", "wait_case", "watch", "timeout",
            "retry", "repeat", "until", 
	    "to", "in", "exists", "union", "inter", "diff",
	    "false", "true", 
            "from", "from_thread",  "receive", "case", "try", "except",
            "min_time", "default", "query_at",
            "of", "atomic_action"]

declaration_keywords = ["rel", "act", "fun", "tel", "dyn", "def",
                        "mrel", "mfun",
                        "tel_percept",  "tel_action",
                        "tel_start", "tel_atomic",
                        "int", "num"]

multi_line_comment = r"/[*]([^*]|([*][^/]))*[*]/"
comment_to_eol = r'%[^\n]*'
all_commentRE = re.compile(multi_line_comment +"|"+comment_to_eol,
                           re.S | re.MULTILINE)
spacesRE = re.compile(r'\s+', re.S | re.MULTILINE)
wspacesRE = re.compile(r'[ \t]+', re.S | re.MULTILINE)

quoted_atom = r"(^|[^0-9])('([^\n']|')*')"
constant = r"[^_a-zA-Z0-9(]((\+|-)?[0-9]+(\.[0-9]+)?)"

string_pattern = r'"(?:[^"\\]+|\\(?:.|))*"'
quote_pattern = r"'(?:[^'\\]+|\\(?:.|))*'"
qulog_nameRE = re.compile(r"[_a-zA-Z][_a-zA-Z0-9]*", re.S | re.MULTILINE)
open_bracRE = re.compile(r"\{|\(|\[", re.S | re.MULTILINE)
close_bracRE = re.compile(r"\}|\)|\]", re.S | re.MULTILINE)
stringRE = re.compile(string_pattern, re.S | re.MULTILINE)
quoteRE = re.compile(quote_pattern, re.S | re.MULTILINE)
numberRE = re.compile(r"[0-9]+(.[0-9]+)?", re.S | re.MULTILINE)

indent_keywords = ["<=", "~>", "~+>", "->", "::=", "::", "==", "=>"]

infix_operators = ["++?", "<>?", "<<", ">>", "\\", "++", "+", "-", "**",
                   "/", "//", "\\/",
                   "/\\", "*", "mod", "to_thread", "to", "from",
                   "from_thread", "for", "with", "<>", "except", "until",
                   "@..", "in", "retry", ":",  "at",
                   "@", "query_at"]

indent_infix_operators = ["=?", "=", "=@", ":=", "+:=", "-:=",  "@<",
                          "@=", "@=<", "@>", "@>=", "\\=", ">=", "<",
                          "=<", ">", "or_while", "commit_while",
                          "min_time", "of"]


quantifiers = ["forall", "exists"]

defn_operators = ["rel", "act", "dyn", "mrel", "mfun",
                  "fun", "tel", "tel_start",
                  "tel_atomic", "tel_percept", "tel_action", "def"]

#indent_keywords = ["<=", "~>", "->", "::", "::=", "==", "=>"]
indent_keywordRE = re.compile('|'.join(indent_keywords), re.S | re.MULTILINE)

prefix_operators = ["not", "once", "call", "do", "doTR", "atomic_action",
                    "repeat", "?", "receive", "try"]

prefix_operatorRE = re.compile(r'not|once|call|do|doTR|atomic_action|\?', re.S | re.MULTILINE)

continuation_operators = [",", ";",  "..",  "&", "|", "||"]
continuation_operatorRE = re.compile(r',|;|\.\.|&|\|\||\|', re.S | re.MULTILINE)


infix_operatorRE = re.compile(r'<<|>>|\+\+\?|\+\+|\+|-|\*\*|/|//|\\/|/\\|\\|\*|mod|to_thread|to|from|from_thread|with|<>\?|<>|except|@..|in|retry|as|:|at|@|query_at', re.S | re.MULTILINE)

indent_infix_operatorRE = re.compile(r'=\?|=@|:=|\+:=|-:=|@<|@=|@=<|@>|@>=|\\=|>=|<|=<|>|=|or_while|commit_while|min_time|of', re.S | re.MULTILINE)

otherRE =re.compile(r'[^ \t\n]', re.S | re.MULTILINE)

qulog_indent_level = 4


def add_face_tag(name, alternates):
    "Return a named group pattern matching list of alternates."
    return "(?P<%s>" % name + "|".join(alternates) + ")"

def make_pat():
    #stringpatt = r'("[^"]*")'
    stringsRE = add_face_tag("stringface", [string_pattern])
    comment1patt = r'%[^\n]*'
    commentRE = add_face_tag("commentface", [comment_to_eol,multi_line_comment])
    variablepatt = r'\b[A-Z_][A-Za-z0-9_]*\b'
    variableRE = add_face_tag("variableface", [variablepatt])
    functorpatt = r'^[a-z][A-Za-z0-9_]*'
    functorRE = add_face_tag("functorface", [functorpatt])
    quoted_atomRE =  add_face_tag("qatomface", [quote_pattern])
    keywordsRE = add_face_tag("keywordface",
                              [r"\b" + k + r"\b" for k in keywords] + \
                              [r"^" + k + r" " for k in declaration_keywords])
    constantRE = add_face_tag("constantface", [constant])
    
    return '|'.join([commentRE,
                     stringsRE,
                     keywordsRE,
                     functorRE,
                     constantRE,
                     quoted_atomRE,
                     variableRE])


prog = re.compile(make_pat(), re.S | re.MULTILINE)

precedence_table = {
    "def": 10,
    "rel": 10,
    "act": 10,
    "tel": 10,
    "fun": 10,
    "tel_percept":10,
    "tel_action":10,
    "dyn":10,
    "mrel":10,
    "mfun":10,
    "commit_while":12,
    "or_while":14,
    "min_time":16,
    "<=":20,
    "->":20,
    "~>":20,
    "~+>":20,
    "::=":20,
    "=>":20,
    "==":20,
    ";":40,
    "repeat":41,
    "until":42,
    "bound":42,
    "try":42,
    "except":42,
    "case":42,
    "wait":42,
    "wait_case":42,
    ',':44,
    '?':45,
    "&":46,
    "forall":47,
    "exists":47,
    "remote_query":47,
    "not":50,
    "once":50,
    "call":50,
    "do":50,
    "doTR":50,
    "of":50,
    "at":50,
    "=":60,
    "=?":60,
    "=@":60,
    "\\=":60,
    ":=":60,
    "+:=":60,
    "-:=":60,
    "@<":60,
    "@=":60,
    "@=<":60,
    "@>":60,
    "@>=":60,
    ">=":60,
    "<":60,
    ">":60,
    "=<":60,
    "in":60,
    "retry":60,
    "++":70,
    "<>":70,
    "@..":70,
    "@":70,
    "+":80,
    "-":80,
    "\\/":80,
    "/\\":80,
    ">>":90,
    "<<":90,
    "mod":90,
    "*":100,
    "/":100,
    "//":100,
    "**":110,
    ":":110,
    "<>?":70,
    "++?":70,
    }

matched_bracket = { "(" : ")", "{" : "}", "[" : "]"}
reverse_matched_bracket = { ")" : "(", "}" : "{", "]" : "["}

def precedence_number(op):
    return precedence_table.get(op, 999)

def eqq_in_context(context):
    for (type, value), _ in context:
        if type == "openbrac":
            return False
        if value == "=?":
            return True
    return False

def has_ge_precedence(op1, op2, context):
    if (op1 == ",") and (op2 == "->"):
        return False
    if op2 == "NL START":
        return op1 != "NL START"
    if op2 == "::" and op1 == "&" and eqq_in_context(context):
        return True
    if op2 == "::":
        if eqq_in_context(context):
            return precedence_number(op1) >= 80
        else:
            return precedence_number(op1) >= 20
    if op1 == "::":
        if eqq_in_context(context):
            return precedence_number(op2) <= 80
        else:
            return precedence_number(op2) <= 20
    if (op2 == "++") and name_in_context("~>", context):
        return precedence_number(op1) >= 20
    if (op1 == "++") and name_in_context("~>", context):
        return precedence_number(op2) <= 20
    return  precedence_number(op1) >= precedence_number(op2)  

def name_in_context(name, context):
    for (n, _), _ in context:
        if n == name:
            return True
    return False

def pop_context(token, context):
    if (not has_ge_precedence(token[1], context[-1][0][1], context)) and \
       context[-1][0][0] != "openbrac":
        context.pop()
        pop_context(token, context)
        
# i1 and i2 are on the same row and i1 >= i2 and both in r.c format
def index_diff(i1, i2):
    (_, c1) = i1.split('.')
    (_, c2) = i2.split('.')
    return int(c1) - int(c2)

class UnmatchedBracketException(Exception):
    def __init__(self, arg):
        self.arg = arg
        




class Editwin(tk.Text):
    def __init__(self, master, parent, **args):
        tk.Text.__init__(self, master, **args)
        self.parent = parent

        for tagid in face_tags:
            self.tag_config(tagid, foreground=face_tags[tagid])
        self.after_idle(self.set_line_and_column)
        
        self.tag_configure("selection", background="#e9e9e9")
        self.tag_configure("bracketerror", background="#ee0000")
        #self.event_add('<<file_new>>', "<Control-x>", "<Control-n>")    
        #self.event_add('<<file_open>>', "<Control-x>", "<Control-o>")    
        #self.event_add('<<file_save>>', "<Control-x>", "<Control-s>")    
        #self.event_add('<<file_save_as>>', "<Control-x>", "<Control-w>")
        self.event_add('<<exit>>', "<Control-x>", "<Control-c>")
        
        self.bind("<Control-x>", self.cut)
        self.bind("<Control-c>", self.copy)
        self.bind("<Control-v>", self.paste)
        
        self.bind("<Tab>", self.tab_pressed)   
        self.bind("<Control-n>", self.parent.file_new)
        self.bind("<Control-o>", self.parent.file_open)
        self.bind("<Control-s>", self.parent.file_save)
        self.bind('<Control-W>', self.parent.file_save_as)
        self.bind('<<exit>>', self.parent.file_quit)
        self.bind("<Control-Y>", self.redo)
        self.bind("<Control-Z>", self.undo)
        #self.bind("<KeyRelease>", self.key_released)
        self.bind("<Double-Button-1>", self.check_brackets)

        self.bind("<<set-line-and-column>>", self.set_line_and_column)
        self.event_add("<<set-line-and-column>>",
                            "<KeyRelease>", "<ButtonRelease>")
        
        self.bind("<<clear-highlight>>", self.clear_highlight)
        self.event_add("<<clear-highlight>>",
                            "<KeyPress>", "<ButtonPress>")
 
        self.edit_modified(False)
        self.find_start = None
        self.find_end = None

    def clear_highlight(self, event=None):
        self.tag_remove("selection", 1.0, "end")
        self.tag_remove("bracketerror", 1.0, "end")
        self.parent.status_label.config(text="")
        
    def set_line_and_column(self, event=None):
        if "KeyRelease" in repr(event):
            self.key_released(event)
        line, column = self.index(tk.INSERT).split('.')
        self.parent.rc_label.config(text= 'Line: {0} Col: {1}'.format(line, column))

    def forward_bracket_match(self, index):
        c = self.get(index)
        bracket_stack = [(index, c)]
        i = self.index(index)
        while self.compare(i, "<", tk.END):
            indc = self.tag_nextrange("commentface", i)
            inds = self.tag_nextrange("stringface", i)
            indq = self.tag_nextrange("qatomface", i)
            if indc and indc[0] == i:
                i = indc[1]
            elif inds and inds[0] == i:
                i = inds[1]
            elif indq and indq[0] == i:
                i = indq[1]
            else:
                i = self.index(i + "+1c")
                
            c = self.get(i)
            if c in matched_bracket:
                bracket_stack.append((i,c))
            elif c in reverse_matched_bracket:
                i0, c0 = bracket_stack.pop()
                if c != matched_bracket[c0]:
                    raise UnmatchedBracketException((c0, i0, c, i))
                if bracket_stack == []:
                    return i
        i0, c0 = bracket_stack.pop()
        raise UnmatchedBracketException(("", 0, c0, i0))

    def backward_bracket_match(self, index):
        c = self.get(index)
        bracket_stack = [(index, c)]
        i = self.index(index)
        while self.compare(i, ">", "1.0"):
            indc = self.tag_prevrange("commentface", i)
            inds = self.tag_prevrange("stringface", i)
            indq = self.tag_prevrange("qatomface", i)
            if indc and self.compare(indc[1], '==',  i):
                i = indc[0]
            elif inds and self.compare(inds[1], '==',  i):
                i = inds[0]
            elif indq and self.compare(indq[1], '==',  i):
                i = indq[0]
            else:     
                i = self.index(i + "-1c")
            c = self.get(i)
            if c in reverse_matched_bracket:
                bracket_stack.append((i,c))
            elif c in matched_bracket:
                i0, c0 = bracket_stack.pop()
                if c0 != matched_bracket[c]:
                    raise UnmatchedBracketException((c, i, c0, i0))
                if bracket_stack == []:
                    return i
        i0, c0 = bracket_stack.pop()
        raise UnmatchedBracketException(("", 0, c0, i0))

    def check_brackets(self, e):
        self.tag_remove("selection", 1.0, "end")
        index = self.index("@%d,%d" % (e.x, e.y))
        tags = self.tag_names(index)
        if "commentface" in tags or "stringface" in tags or "qatomface" in tags:
            return "break"
        c = self.get(index)
        try:
            if c in ['(', '{', '[']:
                next_index = self.forward_bracket_match(index)
                self.tag_add("selection", index, next_index+"+1c")
                return "break"
            elif c in [')', '}', ']']:
                prev_index = self.backward_bracket_match(index)
                self.tag_add("selection", prev_index, index+"+1c")
                return "break"
        except UnmatchedBracketException as e:
            b1, i1, b2, i2 = e.arg
            if (b1 == ''):
                self.parent.status_label.config(text="Unmatched '{}'".format(b2), fg='red')
                self.tag_add("bracketerror", i2, i2+"+1c")
            else:
                self.parent.status_label.config(text="Mismatched '{0}' and '{1}'".format(b1,b2),fg='red')
                self.tag_add("bracketerror", i1, i1+"+1c")
                self.tag_add("bracketerror", i2, i2+"+1c")
            return "break"


    def clear(self):
        self.delete(1.0, "end")
        self.edit_modified(False)
        self.edit_reset()

    def load_text(self, fileContents):
        self.delete(1.0, "end")
        self.insert(1.0, fileContents)
        self.colorize_range("1.0",tk.END)
        self.edit_modified(False)
        self.mark_set('insert', '1.0')

    def save_text(self):
        text = self.get(1.0, "end-1c")
        self.edit_modified(False)
        return text

    def undo(self, event=None):
        try:
            self.edit_undo()
            self.update_colours("1.0",tk.END)
        except:
            pass
        #return 'break'
        
    def redo(self, event=None):
        try:
            self.edit_redo()
            self.update_colours("1.0",tk.END)
        except:
            pass
        #return 'break'

    def find(self, search, forward,replace):
        if search == '':
            return
        self.tag_remove("selection", 1.0, "end")
        index = self.search(search, tk.INSERT, backwards=not forward)
        size = len(search)
        if index:
            self.see(index)
            end_index = index + "+%dc"%size
            self.tag_add("selection", index, end_index)
            if forward:
                self.mark_set('insert', end_index)
            else:
                self.mark_set('insert', index)
            if replace:
                self.find_start = index
                self.find_end = end_index
            else:
                self.find_start = None
                self.find_end = None
        else:
            self.find_start = None
            self.find_end = None



    def search_replace(self, replace):
        if self.find_start is None or replace == '':
            self.find_start = None
            self.find_end = None
            return
        self.delete(self.find_start, self.find_end)
        self.insert(self.find_start, replace)
        self.find_start = None
        self.find_end = None
    
    def tab_pressed(self, e):
        indent, _ = self.compute_indent_level(tk.INSERT)
        self.indent_line_to(indent, tk.INSERT)
        return "break"
    
    def key_released(self, e):
        (start_index, end_index) = self.recolourize_range(tk.INSERT, tk.INSERT)
        self.removecolors_range(start_index, end_index)
        self.colorize_range(start_index, end_index)
        if e.keysym == 'Return':
            #self.insert(tk.INSERT, ' ')
            indent, _ = self.compute_indent_level(tk.INSERT)
            self.indent_line_to(indent, tk.INSERT)

    def update_colours(self, start, end):
        start_index, end_index = self.recolourize_range(start, end)
        self.removecolors_range(start_index, end_index)
        self.colorize_range(start_index, end_index)
        
    def comment_region(self):
        start = self.index("sel.first")
        end = self.index("sel.last")
        start_row = int(start.split('.')[0])
        end_row = int(end.split('.')[0])
        for i in range(start_row, end_row+1):
            self.insert("%d.0"%i, "% ")
        self.update_colours("%d.0"%start_row, "%d.0"%(end_row+1))


    def uncomment_region(self):
        start = self.index("sel.first")
        end = self.index("sel.last")
        start_row = int(start.split('.')[0])
        end_row = int(end.split('.')[0])
        for i in range(start_row, end_row+1):
            ind = "%d.0"%i
            if self.get(ind, ind+"+2c") == '% ':
                self.delete(ind, ind+"+2c")
        self.update_colours("%d.0"%start_row, "%d.0"%(end_row+1))
        start_index, end_index = self.recolourize_range("%d.0"%start_row, "%d.0"%(end_row+1))
        self.removecolors_range(start_index, end_index)
        self.colorize_range(start_index, end_index)
    
    def copy(self, event=None):
        self.clipboard_clear()
        text = self.get("sel.first", "sel.last")
        self.clipboard_append(text)
        return 'break'
        

    def cut(self, event=None):
        self.copy()
        start = self.index("sel.first")
        end = self.index("sel.last")
        self.delete("sel.first", "sel.last")
        self.update_colours(start, end)
        return 'break'
        
    def paste(self, event=None):
        text = self.selection_get(selection='CLIPBOARD')
        self.insert('insert', text)
        self.update_colours('insert', 'insert'+'+%dc'%len(text))
        return 'break'
        
    def recolourize_range(self, start, end):
        forward_end_min = tk.END
        for tag in face_tags:
            i = self.tag_nextrange(tag, end)
            if i:
                last = i[1]
                if self.compare(last, '<', forward_end_min):
                    forward_end_min = last
        backward_end_max = "1.0"
        for tag in face_tags:
            i = self.tag_prevrange(tag, start)
            if i:
                first = i[0]
                if self.compare(first, '>', backward_end_max):
                    backward_end_max  = first
        return (backward_end_max, forward_end_min)
        
    def colorize_range(self, start, end):
        chars = self.get(start, end)
        m = prog.search(chars)
        while m:
            for key, value in m.groupdict().items():
                if value:
                    a, b = m.span(key)
                    self.tag_add(key,
                                 start + "+%dc" % a,
                                 start + "+%dc" % b)
  
            m = prog.search(chars, m.end())

    def removecolors_range(self, start, end):
        for tag in face_tags:
            self.tag_remove(tag, start, end)

    def indent_line_to(self, indent, index):
        #index = tk.INSERT
        begin = self.index(index + " linestart") #beginning_of_line(index)
        text = self.get(begin, tk.END)
        space_match = wspacesRE.match(text)
        if space_match:
            end = space_match.span()[1]
            if end == indent:
                # no change
                return index
            delta = indent - end
        else:
            delta = indent
        if delta < 0:
            end_index = self.index(begin + "+%dc"%-delta)
            self.delete(begin, end_index)
            return self.index(index + "-%dc"%-delta)
        else:
            self.insert(begin, delta*' ')
            return self.index(index + "+%dc"%delta)
 
            
    def indent_all(self):
        index = '1.0'
        self.compute_indent_level(index, True)
        #index = tk.END
        #index = self.index(index + " -1 line")
        #while self.compare(index, '>', '1.0'):
        #    indent, start_index = self.compute_indent_level(index, True)
        #    #self.indent_line_to(indent, index)
        #    index = self.index(start_index + " -1 line")

    def back_to_previous_start(self, index):
        while self.compare(index, ">", "1.0"):
            if "commentface" in self.tag_names(index):
                index = self.tag_prevrange("commentface", index)[0]
                index = self.index(index + " linestart")
            else:
                i = self.search("^[a-z]", index+"+1c",
                                       backwards=True, regexp=True)
                if (i == index):
                    return index
            i = self.index(index)
            r = int(i.split('.')[0])
            if r == 1 :
                return "1.0"
            index = "%d.0"%(r-1)
        return index

    def skip_comment_spaces(self,index):
        text = self.get(index, tk.END)
        offset = 0
        while True:
            comment_match = all_commentRE.match(text)
            if comment_match:
                end = comment_match.span()[1]
                offset += end
                text = text[end:]
                continue
            space_match = spacesRE.match(text)
            if space_match:
                end = space_match.span()[1]
                offset += end
                text = text[end:]
                continue
            break
        return self.index(index + "+%dc"%offset)
            
   
    def next_token(self, index):
        #index = self.skip_comment_spaces(index)
        text = self.get(index, tk.END)
        #print('nt', index, text[:20])
        name_match = qulog_nameRE.match(text)
        ob_match = open_bracRE.match(text)
        if ob_match:
            brac = ob_match.group()
            end = ob_match.span()[1]
            return (("openbrac", brac), index + "+%dc"%end)
        cb_match = close_bracRE.match(text)
        if cb_match:
            brac = cb_match.group()
            end = cb_match.span()[1]
            return (("closebrac", brac), index + "+%dc"%end)
        if name_match:  # name found
            name =  name_match.group()
            end = name_match.span()[1]
            if name in indent_keywords:
                token = ("indent_operator", name)
            elif name in infix_operators:
                token = ("infix_operator", name)
            elif name in indent_infix_operators:
                token = ("indent_infix_operator", name)
            elif name in prefix_operators:
                token = ("prefix_operator", name)
            elif name in quantifiers:
                token = ("quantifier", name)
            elif name in defn_operators:
                token = ("defn_operator", name)
            else:
                token = ("name", name)
            return (token, index + "+%dc"%end)
        string_match = stringRE.match(text)
        if string_match:
            string = string_match.group()
            end = string_match.span()[1]
            return (("string", string), index + "+%dc"%end)
        quote_match = quoteRE.match(text)
        if quote_match:
            quote = quote_match.group()
            end = quote_match.span()[1]
            return (("quote", quote), index + "+%dc"%end)
        number_match = numberRE.match(text)
        if number_match:
            number = number_match.group()
            end = number_match.span()[1]
            return (("number", number), index + "+%dc"%end)
        indent_keyword_match = indent_keywordRE.match(text)
        if indent_keyword_match:
            indent_keyword = indent_keyword_match.group()
            end = indent_keyword_match.span()[1]
            return (("indent_operator", indent_keyword), index + "+%dc"%end)
        prefix_operator_match = prefix_operatorRE.match(text)
        if prefix_operator_match:
            prefix_operator = prefix_operator_match.group()
            end = prefix_operator_match.span()[1]
            return (("prefix_operator", prefix_operator), index + "+%dc"%end)
        continuation_operator_match = continuation_operatorRE.match(text)
        if continuation_operator_match:
            continuation_operator = continuation_operator_match.group()
            end = continuation_operator_match.span()[1]
            return (("continuation_operator", continuation_operator), index + "+%dc"%end) 
        infix_operator_match = infix_operatorRE.match(text)
        if infix_operator_match:
            infix_operator = infix_operator_match.group()
            end = infix_operator_match.span()[1]
            return (("infix_operator", infix_operator), index + "+%dc"%end)
        indent_infix_operator_match = indent_infix_operatorRE.match(text)
        if indent_infix_operator_match:
            indent_infix_operator = indent_infix_operator_match.group()
            end = indent_infix_operator_match.span()[1]
            return (("indent_infix_operator", indent_infix_operator), index + "+%dc"%end)
        other_match = otherRE.match(text)
        if other_match:
            other = other_match.group()
            end = other_match.span()[1]
            return (("other", other), index + "+%dc"%end)
        return (("default","default"), index+"+1c")

    def at_start_of_line(self, index):
        cindex = self.index(index)
        row = cindex.split('.')[0]
        start_of_line = row+'.0'
        start_of_line = self.skip_comment_spaces(start_of_line)
        (_, nindex) = self.next_token(start_of_line)
        nindex = self.index(nindex)
        return self.compare(index, '==', nindex)
    
    def no_more_token_on_line(self, index):
        cindex = self.index(index)
        row = cindex.split('.')[0]
        cindex = self.skip_comment_spaces(cindex)
        (_, nindex) = self.next_token(cindex)
        nindex = self.index(nindex)
        nrow = nindex.split('.')[0]
        return row != nrow
    
    def compute_indent_level(self, start_index, indent_all=False):
        if indent_all:
            line_pos = tk.END
            index = start_index
        else:
            line_pos = self.index(start_index + " linestart")
            start_index = self.back_to_previous_start(line_pos)
            index = start_index
        context = [(("NL START", "NL START"), 0)]
        curr_indent = 0
        token = ("", "")
        prev_is_op = False
        prev_begin = self.index(index + " linestart")
        while self.compare(index, "<", line_pos):
            index = self.skip_comment_spaces(index)
            begin = self.index(index + " linestart")
            if indent_all and self.compare(index, '==', begin):
                context = [(("NL START", "NL START"), 0)]
                curr_indent = 0
                token = ("", "")
                prev_is_op = False
                prev_begin = self.index(index + " linestart")               
            (token, index) = self.next_token(index)
            index = self.index(index)
            #print(token, index, line_pos)
            curr_indent = context[-1][1]
            begin = self.index(index + " linestart")
            if self.compare(begin,  ">",  line_pos):
                token = ("default", "default")
            #print("index", index, "line_pos", line_pos, "token", token, "indent", curr_indent, "prev is op", prev_is_op, "context", context)

            token_type, token_value = token

            if token_type == "defn_operator":
                curr_indent = qulog_indent_level
                context.append((token, curr_indent))
                prev_is_op = True

            elif token_type == "continuation_operator":
                pop_context(token, context)
                context.append((token, context[-1][1]))
                curr_indent = context[-1][1]
                prev_is_op = True

            elif token_type == "indent_operator":
                pop_context(token, context)
                curr_indent = context[-1][1] + qulog_indent_level
                if self.at_start_of_line(index):
                    context.append((token, curr_indent + qulog_indent_level))
                else:
                     context.append((token, curr_indent))
                prev_is_op = True

            elif (token_value == "++") and name_in_context("~>", context):
                pop_context(token, context)
                curr_index = context[-1][1] + qulog_indent_level
                if self.at_start_of_line(index):
                    context.append((token, curr_indent + qulog_indent_level))
                else:
                     context.append((token, curr_indent))
                prev_is_op = True

            elif (token_value == "until") and self.at_start_of_line(index):
                pop_context(token, context)
                context.append((token, context[-1][1]))
                curr_indent = context[-1][1] - qulog_indent_level
                prev_is_op = True
                 
            elif token_type == "infix_operator":
                pop_context(token, context)
                context.append((token, context[-1][1]))
                curr_indent = context[-1][1]
                prev_is_op = True
                
            elif token_type == "indent_infix_operator":
                pop_context(token, context)
                curr_indent = context[-1][1] + qulog_indent_level
                if self.at_start_of_line(index):
                    context.append((token, curr_indent + qulog_indent_level))
                else:
                     context.append((token, curr_indent))
                prev_is_op = True
                
            elif token_type == "prefix_operator":
                if not prev_is_op:
                    if name_in_context("bound", context):
                        (kind, _), _ = context.pop()
                        while kind != "bound":
                            (kind, _), _ = context.pop()
                    else:
                       (kind,_), _ = context[-1]
                       while (kind != "openbrac") and (kind != "NL START"):
                           context.pop()
                           (kind,_), _ = context[-1]
                curr_indent = context[-1][1]
                pop_context(token, context)
                context.append((token, context[-1][1] + qulog_indent_level))
                prev_is_op = True
        
            elif token_type == "quantifier":            
                context.append((token, curr_indent + qulog_indent_level))
                diff = index_diff(self.index(index),
                                  self.index(begin))
                context.append((("bound", "bound"), diff + 1))
                               
            elif token_type == "openbrac" and\
                 (context[-1][0][1] == "repeat" or\
                  context[-1][0][1] == "try" or\
                  context[-1][0][1] == "receive" or\
                  context[-1][0][1] == "until"):
                if self.no_more_token_on_line(index):
                    context.append((token, context[-1][1]))
                else:
                    diff = index_diff(self.index(index),
                                      self.index(begin))
                    context.append((token, diff))
                    
                                   
                
            elif token_type == "openbrac":
                prev_is_op = True
                if name_in_context("bound", context):
                    ((kind, _), _) = context.pop()
                    while kind != "bound":
                        ((kind, _), _) = context.pop()
                if self.no_more_token_on_line(index):
                    context.append((token, context[-1][1] + qulog_indent_level))
                else:
                    diff = index_diff(self.index(index),
                                      self.index(begin))
                    context.append((token, diff))
                    
            elif token_type == "closebrac":
                prev_is_op = False
                ((kind, value), indent) = context.pop()
                while (kind != "openbrac") and \
                      (kind != "NL START"):
                    ((kind, value), indent) = context.pop()
                if (kind != "NL START") and \
                   (token[1] == matched_bracket[value]):
                    curr_indent =  indent
                else:
                    curr_indent = 0
                    line_pos = index

            else:
                if not prev_is_op:
                    if name_in_context("bound", context):
                        ((kind, _), _) = context.pop()
                        while kind != "bound":
                            ((kind, _), _) = context.pop()
                    else:
                        ((kind, value), _) = context[-1]
                        while (kind != "openbrac") and \
                              (kind != "NL START"):
                            context.pop()
                            ((kind, value), _) = context[-1]

                curr_indent =  context[-1][1]
                prev_is_op = False
            if prev_begin != begin:
                prev_begin = begin
                if indent_all and (curr_indent > 0):
                    #print(curr_indent, index)
                    index = self.indent_line_to(curr_indent, index)
            #print("context",context)
        return (curr_indent, start_index)
          


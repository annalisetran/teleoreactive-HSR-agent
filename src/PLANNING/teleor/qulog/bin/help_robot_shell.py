ABOUT_TEXT = """
A simple robot shell for experimenting with TR programs.

Authors: 
   Peter Robinson, 
   Keith Clark
"""

HELP_TEXT = """
This is a simple robot shell for testing TR programs. You take on the role of the robot simulating its activators and sensors and periodically sending back sense data as percepts facts to an agent executing some procedure call in the TR program as a task. 

The program is run as 

robot_shell.py shell_name     
or
robot_shell.py -w shell_name

where shell_name is the Pedro name for this process, that is given to the agent when it is launched. The -w switch is used when the TR agent is using the update percept interface (with the percepts always wrapped with r_, f_, u_, or fa_). The -w switch is omitted for TR agent using the all percept interface.

The TR agent must have a percepts handler thread that starts by sending an initialise message to this shell. You should launch this robot shell before you launch the agent.

When the initialise message is received, you will see this the main window. Respond by typing into the bottom edit window a list of percept facts that records, what you, the robot currently perceive. Send them either using RETURN or clicking on the Send button. If you do not respond within 5 seconds the initialise message will be received and displayed again.  

Percepts are entered as a comma separated list (the '[' and ']' are not required - they are added by the tool). 

If the TR agent is using the all percept interface then you should list the percepts as is (i.e. with no wrappers).
On the other hand, if the TR agent is using the update percept interface then you shoud use one of the wrappers below for each of the percepts (as an example we consider a percept with type see(distance, direction))

* r_(see(10, left))  remember see(10, left)
* f_(see(_, _))  forgets the first instance in the belief store of the see fact. 
* u_(see(8, left))  updates the first "matching" percept fact, say see(10, left), to see(8, left)
* fa_(see(_, _)) forgets all instances of the see fact

For the u_ wrapper it is also possible to wrap some arguments with a ! - e.g.
in the above example we could use u_(see(8, !(left))). This is equivalent to
f_(see(_, left)), r_(see(8, left)) - i.e. arguments with a ! wrapper must match
exactly with the equivalent argument in the old percept being updated.

Inputted percepts information is added to a history that can be navigated using the up and down arrow keys. The history can be saved as a text file and later loaded into the tool.

After each entry of new percepts you will usually see a set  of action controls such as turn(left,0.2) displayed as the agent's response. These are actions for you to simulate. After a second or two imagine what  the sensors would detect and send back a new sequence of percepts, or a percepts update sequence that uses the wrapper to indicate that a percept fact should be forgotten.   

The percepts sent are copied into the text box in blue text and any commands from the TR agent appear in black text. If the percepts is not syntactically correct text it appears in the text box as red text.

"""

ABOUT_TEXT = """
A simple agent shell for experimenting with TR programs.

Authors: 
   Peter Robinson, 
   Keith Clark
"""

HELP_TEXT = """
This is a simple agent shell for sending actions (and other messages) to a robot or a robot simulation. You take on the role of a controlling agent deciding what an appropriate control action response should be to a set of percept facts, which record the latest sense data, that you get from the robot displayed in the main window.    

The program is run as 

agent_shell.py ag_name robot_name

where ag_name is the Pedro name for this process,  and robot_name is the Pedro name of the robot interface process or simulation, running on the same host. If this is on a different host, robot_host,  the second argument should be

 robot_name@robot_host

where robot_host could be an IP address. So you could launch the shell using something like

agent_shell.py robo1Ag robo1@192.168.18.16

The agent_shell process  starts by sending an 

initialise 

message  to  the robot interface. You should launch the  interface process before you launch this agent shell.

When the initialise message is received, the robot should send back a list of percept facts that records what it currently perceives. These will be displayed in the main window of the agent shell. 

In the edit window at the bottom, type in a message and hit RETURN or the Send button. If the message is of the form

act1, act2, ..., actn

it will be wrapped into a string

"[act1, act2, act3])"

and sent to the robot interface. It should be that the robot interface understands each of the action descriptions. 

Messages are added to a history that can be navigated using the up and down arrow keys. The history can be saved as a text file and later loaded into the tool.
Messages sent are copied into the text box in blue text and any percepts appread as black text. 

"""

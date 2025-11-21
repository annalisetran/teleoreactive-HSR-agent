"""Testing the Pedro installation."""

from pedroclient import *
#import time

port_str = raw_input("Port number for Pedro (default 4550): ")
if port_str == '':
    port = 4550
else:
    port = int(port_str)

server = raw_input("IP Address of Pedro server (default localhost): ")
if server == '':
    server = 'localhost'

print "Trying Pedro connection ... ",
try:
    me = PedroClient(async=True)
    print 'OK'
except Exception, e:
    print 'FAILED'
    exit()

print "Trying Pedro registration ... ",
if me.register('test1'):
    print 'OK'
else:
    print 'FAILED'
    exit()


print "Trying Pedro subscription ... ",
if me.subscribe("f(X)", "true"):
    print 'OK'
else:
    print 'FAILED'
    exit()

#print "Sleeping for 3 secs..."
#time.sleep(3)

print "Trying Pedro notification ... ",
if me.notify("f(a)"):
    print 'OK'
else:
    print 'FAILED'
    exit()

print "Trying reading notification ... ",
if me.notification_ready():
    if me.get_notification()[0] == "f(a)":
        print 'OK'
    else:
        print 'FAILED: WRONG NOTIFICATION'
        exit()
else:
    print 'FAILED: NO NOTIFICATION READY'
    exit()

other_machine = raw_input("Start pedro_test2.py on this or another machine\nEnter the IP address of the machine on which pedro_test2.pt is running ")

print "Testing P2P message send ... ",
if me.p2p("test2@'"+other_machine+"'", 'hi'):
    print 'OK'
else:
    print 'FAILED'
    exit()

print "Testing reply ... "

term = me.get_term()[0]
if term.args[2].val == "'hi yourself'":
    print 'OK'
else:
    print 'FAILED: WRONG MESSAGE'
    exit()

    


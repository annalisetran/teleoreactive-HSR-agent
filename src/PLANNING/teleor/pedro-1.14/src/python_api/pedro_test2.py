"""For testing Pedro messages."""

from pedroclient import *

me = PedroClient()
me.register('test2')
print "Test2 is ready to receive message - running on", me.machine

term = me.get_term()[0]
sender = term.args[1]
message = term.args[2].val
print 'Received', message, 'from', str(sender)
print 'sending reply'
me.p2p(sender.args[0].val + '@' + sender.args[1].val, "'hi yourself'")

#!/bin/sh
# the next line restarts using wish\
exec wish "$0" "$@"


#   Copyright (C) 2006, 2007, 2008 Peter Robinson
#   Email: pjr@itee.uq.edu.au

#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.

#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.

#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


# This is a quick and dirty example of a script for connecting and
# interacting with Pedro
# To turn this into a "real" program modify the definition
# of get_notification near the end and replace the remainder of the
# code with your application 

# The program is called as
# tcl_eg.tcl -P <portnumber> -M <machine-name/IP address>
# IF the port argument is not given it defaults to 4550
# If the machine name is not given it defaults to localhost
# WARNING: If you are on a network then you should not use localhost
# as the host name

global host
global ack_socket
global data_socket 
global process_name

set host "localhost"
set port 4550
set i 0
# process runtime args
while {$i < $argc} {
    if {[lindex $argv $i] == "-P"} {
        incr i
        set port [lindex $argv $i]
    } elseif {[lindex $argv $i] == "-M" } {
        incr i
        set host [lindex $argv $i]
    }
    incr i
}


# connect on the ack socket
set ack_socket [socket $host $port]
fconfigure $ack_socket -buffering line -translation lf

# read ID string
gets $ack_socket ID

# connect to data socket
set port1 [expr $port + 1]

set data_socket [socket $host $port1]
fconfigure $data_socket -buffering line -translation lf

# send ID on data socket
puts $data_socket $ID

# get status
gets $data_socket status

if {$status != "ok"} {
    close $data_socket
    close $ack_socket
    exit
}
# We have a connection
# Set up file event for data socket
fileevent $data_socket readable [list get_notification]



# make a subscription (the rock is set to 0)
# returns ID is successful 0  otherwise

proc subscribe { term goal } {
    global data_socket
    global ack_socket
    
    puts $data_socket "subscribe($term, ($goal), 0)"
    gets $ack_socket ID
    return $ID
}

# unsubscribe - return 1 or success 0 on failure
proc unsubscribe { id } {
    global data_socket
    global ack_socket
    
    puts $data_socket "unsubscribe($id)"
    gets $ack_socket ID
    return $ID
}

#  send notification - return 1 or success 0 on failure

proc notify { term } {
    global data_socket
    global ack_socket
    
    puts $data_socket "notify($term)"
    gets $ack_socket ID
    return $ID
}

# register name - return 1 or success 0 on failure
proc register { name } {
    global data_socket
    global ack_socket
    global process_name

    puts $data_socket "register($name)"
    gets $ack_socket ACK
    if { $ACK== 1 } {
        set process_name $name
    }
    return $ACK
}
# deregister name - return 1 or success 0 on failure
proc deregister { } {
    global data_socket
    global ack_socket
    global process_name

    puts $data_socket "deregister($process_name)"
    gets $ack_socket ACK
    return $ACK
}

# send peer-to-peer message  - return 1 or success 0 on failure

proc p2p {toaddr term } {
    global data_socket
    global ack_socket
    global process_name
    global host

    puts "p2pmsg($toaddr, $process_name@'$host', $term)"
    puts $data_socket "p2pmsg($toaddr, $process_name@'$host', $term)"
    gets $ack_socket ACK
    return $ACK
}

##############################################################
## Change the following procedure to what you want
## At the moment it simply puts the notification

proc get_notification {} {
    global data_socket
    gets $data_socket line
    puts $line
}

################################################################

# an example subscription
subscribe "X" "true"

# an example notification
notify "f(a)"

# register a name
register "bar"

# send a p2p message
p2p "foo@'$host'" "hi"

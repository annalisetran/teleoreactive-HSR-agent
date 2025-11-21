#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
HSR-B Teleop Controller
---------------------------
Moving around:
   w    
a  s  d

q/e : rotate left/right

CTRL-C to quit
"""

# Key mappings
moveBindings = {
    'w': (1, 0, 0),     # forward
    's': (-1, 0, 0),    # backward
    'a': (0, 1, 0),     # left
    'd': (0, -1, 0),    # right
    'q': (0, 0, 1),     # rotate left
    'e': (0, 0, -1),    # rotate right
}

def getKey():
    """Get keyboard input"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    """Display current velocity settings"""
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Initialize the ROS node
    rospy.init_node('hsrb_teleop_keyboard')
    
    # Create publisher for velocity commands
    pub = rospy.Publisher('hsrb/command_velocity', Twist, queue_size=1)

    # Set movement parameters
    speed = 0.2  # Linear velocity (m/s)
    turn = 0.5   # Angular velocity (rad/s)
    x = 0
    y = 0
    th = 0

    try:
        print(msg)
        print(vels(speed, turn))

        while not rospy.is_shutdown():
            key = getKey()

            # Check for movement keys
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
            elif key == "j":
                speed += 0.1
                rospy.loginfo(f"new speed = {speed}")
                continue
            elif key == 'k':
                speed -= 0.1
                rospy.loginfo(f"new speed = {speed}")
                continue
            # Exit on CTRL-C
            elif key == '\x03':
                break
            
            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            print(f"current twist: {twist}")
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot before exiting
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
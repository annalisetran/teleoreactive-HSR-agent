#!/usr/bin/python3

# Wall following algorithm adapted from
# https://www.theconstructsim.com/wall-follower-algorithm/


from math import atan2
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

move_speed = 0.1
rotate_speed = 0.5

# separating into 5 regions
# 963 scans total
regions_ = {
    'right': {
        'start' : 0, # 0 - 192
        'end' : 120
    },
    'fright': {
        'start' : 121,
        'end' : 384
    },
    'front': {
        'start' : 385,
        'end' : 576
    },
    'fleft': {
        'start' : 577,
        'end' : 842
    },
    'left': {
        'start' : 843, # 769 - 963
        'end' : 963
    },
}

def check_scan_data(scan_range, region, maximum):
    '''
    checks scan data from scan range within the region. returns the minimum value.
    if that value is greater than maximum, returns maximum instead
    '''
    return min(min(scan_range[region['start'] : region['end']]), maximum)


class WallFollowing():

    def __init__(self) -> None:
        
        # init variables
        self.front_block = 0.4
        self.side_block = 0.6
        self.min_block = 0.2 # minimum front distance to stop moving forward and start rotating
        self.min_side_block = 0.3
        self.min_back_block = 0.3
        
        self.curr_pose = 0.0
        self.prev_pose = 0.0

        self.scan_data = {
            'right': 0.0,
            'fright': 0.0,
            'front': 0.0,
            'fleft': 0.0,
            'left': 0.0,
        }

        #init publishers
        self.velocity_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10) # possibly /hsrb/command_velocity

        #init subscribers
        self.laser_scan_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, callback=self.laserScanCallBack, queue_size=10)
        self.odom_sub = rospy.Subscriber('/hsrb/odom', Odometry, callback=self.odomCallBack, queue_size= 10)

        # state machine
        self.state_description = ""
        self.current_state = self.find_wall


    # called when recieving the odom message
    def odomCallBack(self, msg: Odometry):
        siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.curr_pose = atan2(siny, cosy)
    
    # called when recieving laser scan message
    def laserScanCallBack(self, msg: LaserScan):
        for region in self.scan_data.keys():
            self.scan_data[region] = check_scan_data(msg.ranges, regions_[region], msg.range_max)
    

    def updateVelocity(self, linear, angular):
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self.velocity_pub.publish(velocity)

    def find_wall(self):
        self.state_description = "finding a wall"
        self.updateVelocity(move_speed, rotate_speed * -1)
    
    def turn_left(self):
        self.state_description = "turning left"
        self.updateVelocity(move_speed, rotate_speed)

    def follow_wall(self):
        self.state_description = "following the wall"
        self.updateVelocity(move_speed, 0.0)
    
    def rotate_left(self):
        self.state_description = "rotating left"
        self.updateVelocity(move_speed * 0.7, rotate_speed)

    def change_state(self, state):
        '''
        changes the function stored in current state
        state:
        0 - find_wall
        1 - turn_left
        2 - follow_wall
        3 - rotate_left
        '''
        if state == 0:
            self.current_state = self.find_wall
        elif state == 1:
            self.current_state = self.turn_left
        elif state == 2:
            self.current_state = self.follow_wall
        elif state == 3:
            self.current_state = self.rotate_left

        
        self.current_state()


    def controlLoop(self):
        if self.scan_data['front'] < self.min_block or self.scan_data['fright'] < self.min_side_block:
            #print("front and right under min block")
            self.change_state(3)
        elif self.scan_data['front'] > self.front_block and self.scan_data['fright'] > self.front_block and self.scan_data['right'] < self.min_back_block:
            #print("front and right free, back right blocked")
            self.change_state(2)
        elif self.scan_data['front'] < self.front_block:
            #print("front blocked")
            self.change_state(1)
        elif self.scan_data['fleft'] > self.side_block and self.scan_data['fright'] > self.side_block:
            #print("left and right unblocked")
            self.change_state(0)
        elif self.scan_data['fleft'] > self.side_block and self.scan_data['fright'] < self.side_block:
            #print("left unblocked, right blocked")
            self.change_state(2)
        elif self.scan_data['fleft'] < self.side_block and self.scan_data['fright'] > self.side_block:
            #print("left blocked, right unblocked")
            self.change_state(0)
        elif self.scan_data['fleft'] < self.side_block and self.scan_data['fright'] < self.side_block:
            #print("left and right blocked")
            self.change_state(0)
        else:
            rospy.loginfo("unknown state, scan_data: ",self.scan_data)
        
        # do the current state

        #print(self.state_description)

        


if __name__ == "__main__":
    rospy.init_node("WallFollowing")
    wall_following = WallFollowing()
    
    # running state machine
    while not rospy.is_shutdown():
        wall_following.controlLoop()

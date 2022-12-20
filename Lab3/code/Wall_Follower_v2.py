#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations as tftr
import math


pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)

follow_dir = 0
'''
follow_dir = 0 -> wall finding loop
follow_dir = 1 -> Follow right wall
follow_dir = 2 -> Follow left wall
'''
state = 0
'''
state = 0 -> find wall
state = 1 -> move ahead
state = 2 -> turn right
state = 3 -> turn left
state = 4 -> turn diagonal right
state = 5 -> turn diagonal left
'''


def callback(msg):
    global DIR

    ranges = msg.ranges
    radar_len=len(ranges)

    front_list = ranges[int(345*radar_len/360):radar_len] #+ ranges[0:int(15*radar_len/360)]
    front_list = front_list.remove(0.0)
    left_list = ranges[int(70*radar_len/360):int(90*radar_len/360)]
    left_list = left_list.remove(0.0)
    right_list = ranges[int(270*radar_len/360):int(290*radar_len/360)]
    right_list = right_list.remove(0.0)
    print(type(front_list))
    DIR = {
    'left': min(left_list),
    'front': min(front_list), 
    'right': min(right_list),
}

    print(front_list)
    movement()


def movement():
    global follow_dir, state

    b = 1  # maximum threshold distance
    a = 0.1  # minimum threshold distance
    velocity = Twist()  # Odometry call for velocity
    linear_x = 0  # Odometry message for linear velocity will be called here.
    angular_z = 0  # Odometry message for angular velocity will be called here.

    rospy.loginfo("follow_direction {f}".format(f=follow_dir))  # This will indicate the direction of wall to follow.

    if DIR['front'] > b and DIR['left'] > b and DIR['right'] > b:  # Loop 1
        if follow_dir == 0:
            state = 0 #Move forward till you find a wall
        rospy.loginfo("Finding a wall")
    elif follow_dir == 0: #Setting the direction of wall to follow
        if DIR['left'] < b:
            state = 2
            follow_dir = 2
            rospy.loginfo("following left wall")
        elif DIR['right'] < b:
            state = 3
            follow_dir = 1
            rospy.loginfo("following right wall")
        else:
            if follow_dir == 0:
                state = 3 #Go right on the very first time you find the wall
            elif follow_dir == 1:
                state = 3 #When you follow right wall and find right turn, go right
            elif follow_dir == 2:
                state = 4 #When you follow left wall and find left turn, go left
            # rospy.loginfo("follow direction not set going right") - x
    elif DIR['left'] < a or DIR['right'] < a:
        rospy.loginfo("Too Close")
        if follow_dir == 1:
            state = 3 #When you follow right wall and get too close, go right
        elif follow_dir == 2:
            state = 4 #When you follow left wall and get too close, go left
    else:
        rospy.loginfo("Running")

    if follow_dir == 2:  # Algorithm for left wall follower
        if DIR['left'] > b and DIR['front'] > a:
            state = 5 #You go diagonally left!!!!!!!!!!!!!!!!!
        elif DIR['left'] < b and DIR['front'] > a:
            state = 1
        elif DIR['left'] < b and DIR['front'] < a:
            state = 3
        else:
            rospy.loginfo("follow left wall is not running")
    elif follow_dir == 1:  # Algorithm for right wall follower
        if DIR['right'] > b and DIR['front'] > a:
            state = 4 #You go diagonally right!!!!!!!!!!!!!!
        elif DIR['right'] < b and DIR['front'] > a:
            state = 1
        elif DIR['right'] < b and DIR['front'] > a:
            state = 3
        elif DIR['right'] < b and DIR['front'] < a:
            state = 3 #You turn left!!!!!!!!!!!!!!!!!!!!!!!
        else:
            rospy.loginfo("follow right wall is not running")

def getAngle(msg):
    global cur_pos
    cur_r = msg.pose.pose.orientation
    cur_pos1 = tftr.euler_from_quaternion((cur_r.x, cur_r.y, cur_r.z, cur_r.w))
    cur_pos = cur_pos1[2]

def move_diag_left():
    velocity = Twist()
    velocity.linear.x = 0.1
    velocity.angular.z = 0.3
    return velocity

def move_diag_right():
    velocity = Twist()
    velocity.linear.x = 0.1
    velocity.angular.z = -0.3
    return velocity

def move_ahead():
    velocity = Twist()
    velocity.linear.x = 0.3
    velocity.angular.z = 0
    return velocity

def turn_right():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = -0.3
    return velocity

def turn_left():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = 0.3
    return velocity

def find_wall():
    velocity = Twist()
    velocity.linear.x = 0.3
    velocity.angular.z = 0
    return velocity

def stop_robot():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = 0
    pub_cmd_vel.publish(velocity)

def publish_vel():
    global pub_cmd_vel, state

    rospy.init_node('follow_wall')
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.Subscriber("/odom", Odometry, getAngle)

    while not rospy.is_shutdown():

        velocity = Twist()
        if state == 0:
            velocity = find_wall()
        elif state == 1:
            velocity = move_ahead()
        elif state == 2:
            velocity = turn_right()
        elif state == 3:
            velocity = turn_left()
        elif state == 4:
            velocity = move_diag_right()
        elif state == 5:
            velocity = move_diag_left()
        else:
            rospy.logerr('Unknown state!')

        pub_cmd_vel.publish(velocity)
    rospy.spin()

if __name__ == '__main__':
    publish_vel()
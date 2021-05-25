#! /usr/bin/env python

import time
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

pose = 0
temp = 0

def callback(msg):
    global pose 
    pose = msg.pose.pose

def callba(data):
    global temp 
    temp = data.data
    #rospy.loginfo(rospy.get_caller_id() + 'Temp : %s', data.data)


def listen():
    start = time.time()
    seconds = 2
    while True:
        rospy.Subscriber('odom', Odometry, callback)
        rospy.Subscriber('chatter', String, callba)
    
        cur = time.time()
        elapse = cur - start
        if elapse >= seconds:
            print pose, temp
            break

if __name__ == '__main__':
    while True:
        rospy.init_node('check_odometry')
        listen()
        print("Breaker")
        
    


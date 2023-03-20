#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(msg): # callback which is called everytime there is a message
    print msg.data

rospy.init_node('topic_subscriber')

# Send the subscribe info to roscore, if channel doesnt exist, it simply waits till a message comes on this channel
sub = rospy.Subscriber('counter', Int32, callback) 

rospy.spin() # give back control to ROS

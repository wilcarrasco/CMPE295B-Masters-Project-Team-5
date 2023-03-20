#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher')

 # announcing that the topic name is "counter" and data type is Int32
pub = rospy.Publisher('counter', Int32, queue_size=10)

rate = rospy.Rate(2) # defining the frequency of messages in Hz (msgs per sec)

count = 0
while not rospy.is_shutdown():
    pub.publish(count) # Sending the actual message
    count += 1
    rate.sleep() # uses previously set value to determine delay

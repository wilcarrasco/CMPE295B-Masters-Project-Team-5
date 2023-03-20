#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

#[sensor_msgs/LaserScan]:
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#float32 angle_min
#float32 angle_max
#float32 angle_increment
#float32 time_increment
#float32 scan_time
#float32 range_min
#float32 range_max
#float32[] ranges
#float32[] intensities

def lidar_callback(msg):
        #rospy.loginfo(msg)
        print(msg.header.stamp)
        #print(len(msg.ranges))
        #print(len(msg.intensities))

if __name__ == "__main__":
        rospy.init_node("my_rplidar_listener")

        sub = rospy.Subscriber("/scan", LaserScan, callback=lidar_callback)

        rospy.loginfo("Node has been started")

        rospy.spin()


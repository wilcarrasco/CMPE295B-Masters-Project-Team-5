#!/usr/bin/env python
import rospy
import struct
import argparse
import numpy as np
import matplotlib.pyplot    as plt
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg        import PointCloud2
from darknet_ros_msgs.msg   import BoundingBox, BoundingBoxes
from geometry_msgs.msg      import Vector3

###
# ROS CLASS IMPLEMENTATION
# https://answers.ros.org/question/225008/python-class-for-subscribing-data-for-several-topics/
#

#############################################
# Global Variables                          #
#############################################
bbox    = BoundingBox()
pre_bbox= BoundingBox()
pc      = PointCloud2()
center  = Vector3()
point   = Vector3()
points  = []
depth   = 0.0

#def bbox_callback(msg):
#    num_box = len(msg.bounding_boxes)
#    if num_box>0:
#        b         = msg.bounding_boxes[0]
#        bbox.xmin = b.xmin
#        bbox.xmax = b.xmax
#        bbox.ymin = b.ymin
#        bbox.ymax = b.ymax

#person=0
#book=73

#bottle=39
#apple=47
#orange=49
consecutive=[0,0,0]
saw_last=[False,False,False]
def bbox_callback(msg):
    num_box = len(msg.bounding_boxes)
    #print(num_box)
    saw_this=[False,False,False]
    if num_box>0:
        for i in range(num_box):
            b         = msg.bounding_boxes[i]
            #print("    " + b.Class + " " + str(b.id) + " " + str(b.probability))
            if b.id==39:
                saw_this[0]=True
                if saw_last[0]==True:
                    consecutive[0] = consecutive[0] + 1
                else:
                    consecutive[0] = 1
                    saw_last[0]=True
            if b.id==47:
                saw_this[1]=True
                if saw_last[1]==True:
                    consecutive[1] = consecutive[1] + 1
                else:
                    consecutive[1] = 1
                    saw_last[1]=True
            if b.id==49:
                saw_this[2]=True
                if saw_last[2]==True:
                    consecutive[2] = consecutive[2] + 1
                else:
                    consecutive[2] = 1
                    saw_last[2]=True
        if saw_this[0]==False:
            saw_last[0]=False
        if saw_this[1]==False:
            saw_last[1]=False
        if saw_this[2]==False:
            saw_last[2]=False
        print(str(consecutive[0]) + " " + str(consecutive[1]) + " " + str(consecutive[2]))


def point_callback(msg):
    global points
    global bbox
    global pre_bbox
    pc.header = msg.header
    pc.height = msg.height
    pc.width  = msg.width
    pc.fields = msg.fields      # channels and their layout
    pc.is_bigendian = msg.is_bigendian
    pc.point_step   = msg.point_step
    pc.row_step     = msg.row_step
    pc.data         = msg.data  #  Actual point data, size is (row_step*height)
    resolution = (pc.height, pc.width)


    if bbox.xmin==pre_bbox.xmin and \
        bbox.xmin==pre_bbox.xmin and \
        bbox.xmin==pre_bbox.xmin and \
        bbox.xmin==pre_bbox.xmin:
        pass
    else:
        points = [  ]
        for p in pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z")):
            points.append(p[2])
        if len(points) == pc.width*pc.height:
            z_points = np.array(points, dtype=np.float32)
            z = z_points.reshape(resolution)

            if not (bbox.xmin==0 and bbox.xmax==0):
                print('Box: {}, {}'.format(bbox.xmin, bbox.xmax))
                z_box = z[bbox.xmin:bbox.xmax, bbox.ymin:bbox.ymax]
                z_value = z_box[~np.isnan(z_box)]

                distance = min(z_value)
                print('Distance: {}'.format(distance))

        pre_bbox.xmin = bbox.xmin
        pre_bbox.xmax = bbox.xmax
        pre_bbox.ymin = bbox.ymin
        pre_bbox.ymax = bbox.ymax


def main(args):
    rospy.init_node('ObjectDepth', anonymous=True)

    point_sub   = rospy.Subscriber('camera/depth_registered/points', PointCloud2, point_callback)
    #bbox_sub    = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, bbox_callback)

    rospy.loginfo("Node has been started")
    rospy.spin()

    #freq = 30
    #rate = rospy.Rate(freq)

    #while not rospy.is_shutdown():
    #    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Output Depth of an Target Object')
    args = parser.parse_args()
    main(args)

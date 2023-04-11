#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, Point
from geometry_msgs.msg import Pose
from transbot_msgs.msg import PWMServo
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
import tf

class LaserImageFusion:

    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/PWMServo', PWMServo, self.servo_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('/map', '/laser', rospy.Time(0), rospy.Duration(4.0))
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.ranges = None
        self.left_edge = None
        self.right_edge = None
        self.servo_pointing = None #figure out how to get this data flowing 

    def servo_callback(self,msg):
        self.servo_pointing = msg.angle

    def laser_callback(self, msg):
        # 0 is front 180 is left, 360 is back, 539 is right 
        self.ranges = msg.ranges
        # print(self.laser_header)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position # need to add in robot angle 
        
    def bbox_callback(self, msg):
        for obj in msg.bounding_boxes:
            if obj.probability > .6:
                self.left_edge = convert_image_to_angle(obj.xmin)
                self.right_edge = convert_image_to_angle(obj.xmax)
    
    def fuse(self):
        if self.ranges is not None and self.left_edge is not None and self.right_edge is not None:

            if self.servo_pointing is not None:
                self.left_edge += self.servo_pointing
                self.right_edge += self.servo_pointing

            ranges, angle = find_ranges(self.ranges, self.left_edge, self.right_edge)
            
            self.place_marker(ranges, angle)
    
    def place_marker(self, ranges, angle):
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/laser', rospy.Time(0))
        p0 = PointStamped()
        p0.point.x = ranges * np.cos((angle+180) * np.pi/180)
        p0.point.y = ranges * np.sin((angle+180) * np.pi/180)
        p0.point.z = 0.0
        p0.header.frame_id = "laser"
        p0.header.stamp = rospy.Time(0)

        try:
            p0 = self.tf_listener.transformPoint('/map', p0)
        except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
            return

        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = p0.point.x
        marker.pose.position.y = p0.point.y
        marker.pose.position.z = p0.point.z
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.marker_pub.publish(marker)


def find_ranges(ranges, l_edge, r_edge):
    l_edge_idx = np.int_(np.floor(l_edge) - 1) # think this needs to be mult by 2
    r_edge_idx = np.int_(np.ceil(r_edge) - 1)

    if r_edge_idx >=0 and l_edge_idx >= 0:
        selected_ranges = ranges[r_edge_idx:l_edge_idx]
    elif r_edge_idx <=0 and l_edge_idx <= 0:
        r_edge_idx = len(ranges) + r_edge_idx + 1
        l_edge_idx = len(ranges) + l_edge_idx + 1
        selected_ranges = ranges[r_edge_idx:l_edge_idx]
    elif r_edge_idx <=0 and l_edge_idx >= 0:
        r_edge_idx = len(ranges) + r_edge_idx + 1
        neg_side = ranges[r_edge_idx:]
        pos_side = ranges[0:l_edge_idx]
        selected_ranges = pos_side + neg_side
    selected_ranges = np.array(selected_ranges)
    selected_ranges[np.isinf(selected_ranges)] = np.nan

    min_range = np.nanmin(selected_ranges)
    mean_angle = np.nanmean([l_edge, r_edge])

    print("Min Range & Mean Angle : ", min_range, mean_angle)

    return min_range, mean_angle
    

def convert_image_to_angle(image_position):

    # define camera intrinsic parameters
    fov = 60 # in degrees
    image_size = (640, 480) # in pixels
    aspect_ratio = image_size[0] / image_size[1]
    f = image_size[0] / (2 * np.tan(np.deg2rad(fov) / 2)) # in pixels
    principal_point = (image_size[0] / 2, image_size[1] / 2) # in pixels

    # define camera extrinsic parameters
    camera_position = np.array([0, 0, 0]) # in meters
    camera_orientation = np.identity(3) # camera is pointed straight ahead

    # convert image position to camera coordinates
    image_coords = np.array([image_position - principal_point[0], principal_point[1] - principal_point[1], f])
    camera_coords = np.linalg.inv(camera_orientation).dot(image_coords)

    # compute angle between camera and object
    object_position = np.array([0, 0, 1]) # object is 1 meter in front of camera
    object_vector = object_position - camera_position
    reference_vector = np.array([0, 1, 0]) # reference vector on camera plane
    angle = np.arccos(np.dot(object_vector, camera_coords) / (np.linalg.norm(object_vector) * np.linalg.norm(camera_coords)))

    # compute sign of angle based on reference vector
    cross = np.cross(reference_vector, camera_coords)
    if np.dot(cross, object_vector) < 0:
        angle *= -1

    # print("Angle between camera and object:", np.rad2deg(angle), image_position)
    return np.rad2deg(angle)

if __name__ == '__main__':
    rospy.init_node('laser_readings')
    fusion = LaserImageFusion()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        fusion.fuse()
        rate.sleep()

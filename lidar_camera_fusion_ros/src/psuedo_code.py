# Pull Data from Yolo, Lidar, and Odometry
rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
rospy.Subscriber('/scan', LaserScan, self.laser_callback)
rospy.Subscriber('/PWMServo', PWMServo, self.servo_callback)
rospy.Subscriber('/odom', Odometry, self.odom_callback)

# Get Left and Right edges of bounding boxes if Probabilty above 0.6
def bbox_callback(self, msg):
    for obj in msg.bounding_boxes:
        if obj.probability > .6:
            self.left_edge = convert_image_to_angle(obj.xmin)
            self.right_edge = convert_image_to_angle(obj.xmax)
            
# Convert left and right edges of bounding box to angles within frame
# Angles can be calculated knowning camera FOV and resolution
def convert_image_to_angle(image_position):
    aspect_ratio = image_size[0] / image_size[1]
    f = image_size[0] / (2 * np.tan(np.deg2rad(fov) / 2)) # in pixels
    principal_point = (image_size[0] / 2, image_size[1] / 2) # in pixels
    angle = np.arccos(np.dot(object_vector, camera_coords) / 
                      (np.linalg.norm(object_vector) * np.linalg.norm(camera_coords)))
    return np.rad2deg(angle)
    
# Find min range and mean angle based on left and right edge angles
def find_ranges(ranges, l_edge, r_edge):
    l_edge_idx = np.int_(np.floor(l_edge) - 1)
    r_edge_idx = np.int_(np.ceil(r_edge) - 1)
    selected_ranges = np.array(selected_ranges)
    min_range = np.nanmin(selected_ranges)
    mean_angle = np.nanmean([l_edge, r_edge])
    print("Min Range & Mean Angle : ", min_range, mean_angle)
    return min_range, mean_angle

# Convert mean angle and min range in lidar frame to map frame
def convert_to_map_frame(ranges, angle):
    (trans, rot) = self.tf_listener.lookupTransform('/map', '/laser', rospy.Time(0))
    p0 = PointStamped()
    p0.point.x = ranges * np.cos((angle+180) * np.pi/180)
    p0.point.y = ranges * np.sin((angle+180) * np.pi/180)

# Place marker on RVIZ map
def place_marker(self, ranges, angle):
      # Create a Marker message
      marker.type = Marker.SPHERE
      marker.action = Marker.ADD
      marker.pose.position.x = p0.point.x
      marker.pose.position.y = p0.point.y
      marker.pose.position.z = p0.point.z
      # Publish the marker
      self.marker_pub.publish(marker)



    

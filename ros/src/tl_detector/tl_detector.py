#!/usr/bin/env python

import tf
import cv2
import yaml
import math
import rospy
import numpy as np
from std_msgs.msg      import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg     import TrafficLightArray, TrafficLight
from styx_msgs.msg     import Lane
from sensor_msgs.msg   import Image
from cv_bridge         import CvBridge
from light_classification.tl_classifier import TLClassifier


STATE_COUNT_THRESHOLD = 3


class TLDetector(object):

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose',           PoseStamped,       self.pose_cb)
        rospy.Subscriber('/base_waypoints',         Lane,              self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color',            Image,             self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config   = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg


    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints


    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        #rospy.logwarn('Light state: ' + str(state) + ' Next light waypoint: ' + str(light_wp))


    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)


    def get_closest_waypoint(self, pose, waypoints):
        closest_dist = float('inf')
        closest_wp = 0
        for i in range(len(waypoints)):
            dist = self.distance(pose.position.x, pose.position.y,
                                 waypoints[i].pose.pose.position.x,
                                 waypoints[i].pose.pose.position.y)
            if dist < closest_dist:
                closest_dist = dist
                closest_wp = i

        return closest_wp


    def get_next_waypoint(self, pose, waypoints):
        closest_wp = self.get_closest_waypoint(pose, waypoints)
        wp_x = waypoints[closest_wp].pose.pose.position.x
        wp_y = waypoints[closest_wp].pose.pose.position.y
        heading = math.atan2( (wp_y-pose.position.y), (wp_x-pose.position.x) )
        angle = abs(pose.position.z-heading)
        if angle > math.pi / 4.0:
            closest_wp += 1

        return closest_wp


    def project_to_image_plane(self, point_in_world):
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # Get principal point (center of image)
        cx = image_width / 2
        cy = image_height / 2

        # Get transform between pose of camera and world frame
        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform('/base_link', '/world', now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform('/base_link', '/world', now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # Use tranform and rotation to calculate 2D position of light in image

        # Create numpy array containing the 3D world point
        object_point = np.array([[point_in_world.x, point_in_world.y, point_in_world.z]])

        # Convert the quaternion returned from lookupTransform into euler rotation
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)
        rvec = np.array([roll,pitch,yaw])
        tvec = np.array(trans)

        # Create the camera matrix from the focal lengths and principal point
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        # Distortion coefficients - currently not available but per slack will be published soon
        dist_coeffs = None

        # Use OpenCV projectPoints to find the corresponding point in image from 3D world point
        img_point, _ = cv2.projectPoints(object_point, rvec, tvec, camera_matrix, dist_coeffs)

        # Cast to int to get a pixel value
        pixels = np.int32(img_point).reshape(-1, 2)
        x = pixels[0][0]
        y = pixels[0][1]

        return (x, y)


    def get_light_state(self, light):
        if not self.has_image:
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, 'rgb8')
        return self.light_classifier.get_classification(cv_image)


    def create_light(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = 'world'

        light.pose = self.create_pose(x, y, z, yaw)
        light.state = state

        return light


    def create_pose(self, x, y, z, yaw=0.0):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi * yaw / 180.0)
        pose.pose.orientation = Quaternion(*q)

        return pose


    def process_traffic_lights(self):
        light = None
        light_positions = self.config['stop_line_positions']
        max_visible_dist = 80.0
        min_dist = float('inf')
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose, self.waypoints.waypoints)

            # Find the closest visible traffic light (if one exists)
            for light_pos in light_positions:
                light_tmp = self.create_light(light_pos[0], light_pos[1], 0.0, 0.0, TrafficLight.UNKNOWN)
                light_position = self.get_closest_waypoint(light_tmp.pose.pose, self.waypoints.waypoints)
                dist = self.distance(self.waypoints.waypoints[car_position].pose.pose.position.x,
                                     self.waypoints.waypoints[car_position].pose.pose.position.y,
                                     self.waypoints.waypoints[light_position].pose.pose.position.x,
                                     self.waypoints.waypoints[light_position].pose.pose.position.y)
                if dist < min_dist and dist < max_visible_dist and car_position < light_position:
                    min_dist = dist
                    light = light_tmp
                    light_wp = light_position

        if light:
            state = self.get_light_state(light)
            return light_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')


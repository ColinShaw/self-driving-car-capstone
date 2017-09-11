#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class VehicleView(object):

    def __init__(self):

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)

    def camera_callback(self, data):

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print e

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


def main():
    vehicle_view_object = VehicleView()
    rospy.init_node('vehicle_view_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__=='__main__':
    main()

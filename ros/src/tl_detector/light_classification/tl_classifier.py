from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction


        lower_black = np.array([200,0,0], dtype = "uint8")
        upper_black = np.array([255,100,100], dtype = "uint8")
        Red_sum = np.sum(cv2.inRange(image, lower_black, upper_black))
        lower_black = np.array([0,170,0], dtype = "uint8")
        upper_black = np.array([150,255,150], dtype = "uint8")
        Green_sum = np.sum(cv2.inRange(image, lower_black, upper_black))
        lower_black = np.array([200,200,0], dtype = "uint8")
        upper_black = np.array([255,255,100], dtype = "uint8")
        Yellow_sum = np.sum(cv2.inRange(image, lower_black, upper_black))

        if (Red_sum < 500) and (Green_sum < 500) and (Yellow_sum < 500): # 500 is a threshold meaning probabbly not a light
            return TrafficLight.UNKNOWN
        elif (Red_sum > Green_sum):
            if (Red_sum > Yellow_sum):
                return TrafficLight.RED
            else:
                return TrafficLight.YELLOW

        elif (Yellow_sum > Green_sum):
            if (Yellow_sum > Red_sum):
                return TrafficLight.YELLOW
            else:
                return TrafficLight.RED
        else:
            return TrafficLight.GREEN


        

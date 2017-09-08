from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import keras
from keras.models import load_model


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = load_model('../../../data_science/models/udacity_vgg_fine_tuning.h5')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        img = cv2.resize(image, (224,224), interpolation=cv2.INER_AREA)
        img = np.array(img, np.float32) / 255.

        pred = self.model.predict(img, batch_size=1)
        # Find class based on prediction index (need to confirm labels are correct order)
        if pred == 0:
            state = TrafficLight.GREEN
        elif pred == 1:
            state = TrafficLight.UNKNOWN # index 1 corresponds to no light
        elif pred == 2:
            state = TrafficLight.RED
        else:
            state = TrafficLight.YELLOW

        return state

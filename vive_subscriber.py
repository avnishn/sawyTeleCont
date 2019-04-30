import rospy
import tf
from enum import Enum

class Vive_Subscriber():
    def __init__(self):
        rospy.init_node('vive_subscriber',anonymous=True)
        self._tfListener = tf.TransformListener()
        self._button_state_left = None
        self._button_state_right = None
        self._base_frame = "/base" 
    
    def _buttonsPressedCallbackLeft(self, data):
        self._button_state_left = data.buttons
        self.buttonAxesLeft = data.axes
    
    def _buttonsPressedCallbackRight(self, data):
        self._button_state_right = data.buttons
        self.buttonAxesRight = data.axes

    def initControllerListener(self):
        rospy.Subscriber("vive_left", Joy, self._buttonsPressedCallbackLeft)
        rospy.Subscriber("vive_right", Joy, self._buttonsPressedCallbackRight)

    def getControllerPositionWRTWorld(self, controller):
        if (controller is False):
            controller_frame = '/left_controller'
        elif (controller is True):
            controller_frame = '/right_controller'
        else:
            raise Exception('Did you specify the proper controller frame?')
        try:
            (trans,rot) = self._tfListener.lookupTransform(self._base_frame, controller_frame, rospy.Time(0))
            return trans,rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    @property
    def left_buttons_state(self):
        return self._button_state_left

    @property
    def right_buttons_state(self):
        return self._button_state_right


#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pid_controller import PIDControllerThreePoints

# ------------------------------------------------------------------------------------------------ #
# controller listens from 2 topics for each controller
# 1) contoller name (eg. vive_left) gives the output of the buttons on each controller
# 2) `controller_name`_as_posestamped gives the location of the vive controller wrt the headset
# 2 types of messages: 1) geometry_msgs/PoseStamped 2) sensor_msgs/Joy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

# ------------------------------------------------------------------------------------------------ #



class sawyerTeleoperation(object):
    def __init__(self):
        rospy.init_node('sawyerTeleoperation',anonymous=True)
        self.position = None
        self.buttonsState = None

        self.initControllerListener()
        self.tfListener = tf.TransformListener()
        self.initMoveIt()

    def initMoveIt(self):
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

        self.eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % self.eef_link

   
    def buttonsPressedCallback(self, data):
        # print(data.axes)
        self.buttonsState = data.buttons
        self.buttonAxes = data.axes

    def initControllerListener(self):
        rospy.Subscriber("vive_left", Joy, self.buttonsPressedCallback)

    def getControllerPositionWRTWorld(self):
        try:
            (trans,rot) = self.tfListener.lookupTransform('/world', '/left_controller', rospy.Time(0))
            self.position = trans
            self.rotation = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def run(self):
        while not rospy.is_shutdown():
            self.getControllerPositionWRTWorld()
            # if((self.buttonsState) and (self.position)):
            #     print("controller position {}, trigger enabled: {}".format(self.position, self.buttonsState[0]))

            if( (self.buttonsState is not None) and (self.buttonsState[0])):
                print("controller position {}, trigger enabled: {}".format(self.position, self.buttonsState[0]))
                self.group.set_random_target()
                plan_msg = self.group.plan()
                self.group.execute(plan_msg=plan_msg, wait=False)
                rospy.sleep(5)
                
            



 # 10hz
# while not rospy.is_shutdown():
#    # store old sawyer position
#    # store "goal position" initially the same as sawyer old position
#    # if (saftey on controller engaged):
#         # read current position, use pid controller, update old sawyer position
#         # publish "old sawyer position"
#         # keep sampling pid controller and updating old sawyer position till 
#         # we reach the goal positon.
#         # update goal position if htc controller has moved

        
#    r.sleep()

if __name__ == "__main__":
    st = sawyerTeleoperation()
    r = rospy.Rate(250)
    st.run()
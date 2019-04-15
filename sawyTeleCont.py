#!/usr/bin/env python
import sys
import copy
import rospy
import numpy as np
import tf

# from pid_controller import PIDControllerThreePoints

# ------------------------------------------------------------------------------------------------ #
# controller listens from 2 topics for each controller
# 1) contoller name (eg. vive_left) gives the output of the buttons on each controller
# 2) `controller_name`_as_posestamped gives the location of the vive controller wrt the headset
# 2 types of messages: 1) geometry_msgs/PoseStamped 2) sensor_msgs/Joy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Joy
import intera_interface
from pid_controller import PIDControllerThreePoints

# ------------------------------------------------------------------------------------------------ #

class sawyerTeleoperation(object):
    def __init__(self):
        rospy.init_node('sawyerTeleoperation',anonymous=True)
        
        self.position = None
        self.buttonsState = None

        self.initControllerListener()
        self.tfListener = tf.TransformListener()
        self.limb = intera_interface.Limb('right')
        self.limb.set_joint_position_speed(speed=0.1)
        self.initial_pos = self.limb.endpoint_pose()
        self.startingRobotPosition = self.limb.endpoint_pose()
        self.PID = PIDControllerThreePoints(kp=0.2, kd=0.6)
        # wpose = self.group.get_current_pose().pose

    def buttonsPressedCallback(self, data):
        # print(data.axes)
        self.buttonsState = data.buttons
        self.buttonAxes = data.axes

    def initControllerListener(self):
        rospy.Subscriber("vive_left", Joy, self.buttonsPressedCallback)

    def getControllerPositionWRTWorld(self):
        try:
            (trans,rot) = self.tfListener.lookupTransform('/world', '/left_controller', rospy.Time(0))
            return trans,rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    @property
    def robotPosition(self):
        pos, _ = self.tfListener.lookupTransform("base", "right_gripper_tip", rospy.Time(0)) 
        return np.array(pos)

    @property
    def robotGripperOri(self):
        ori = self.limb.endpoint_pose()['orientation']
        return ori

    def run(self):
        self.limb.move_to_neutral()
        msg = "=========================starting========================="
        rospy.loginfo(msg)
        startControllerPosition = None
        self.r = rospy.Rate(0.45)
        while not rospy.is_shutdown():

            if( (self.buttonsState is not None) and not(self.buttonsState[0])):
                startControllerPosition, _ = self.getControllerPositionWRTWorld()

            if( (self.buttonsState is not None) and (self.buttonsState[0])):
                currentControllerPosition, _ = self.getControllerPositionWRTWorld()

                # displacement = currControllerPos - startControllerPos
                # updated position = displacement + startingRobotPos
                # pointToMoveTo = pidcontroler(updatedPosition)
                # get joint angles using IK solver
                # send to robot
                
                displacement = np.subtract(np.asarray(currentControllerPosition), np.asarray(startControllerPosition))
                for x in displacement:
                    x *= 1.5
                updatedPosition = np.add(displacement, self.robotPosition)
                startControllerPosition = currentControllerPosition

                pid_pos = self.PID.update(self.robotPosition, updatedPosition, rospy.get_time())

                # construct message
                pose_msg = Pose()
                pose_msg.position = Point(pid_pos[0], pid_pos[1], pid_pos[2])
                pose_msg.orientation = self.robotGripperOri

                joint_angles = self.limb.ik_request(pose_msg, "right_gripper_tip")
                if joint_angles:
                    self.limb.move_to_joint_positions(joint_angles, timeout=2)

                msg = "robotPosition:{}, displacement:{}, updatedPosition:{}, pid_pos:{}".format(self.robotPosition, displacement, updatedPosition, pid_pos)
                rospy.loginfo(msg)
                self.r.sleep()


if __name__ == "__main__":
    st = sawyerTeleoperation()
    st.run()

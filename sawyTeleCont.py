#!/usr/bin/env python
import sys
import copy
import rospy
import numpy as np
import tf
import time
import copy

# from pid_controller import PIDControllerThreePoints

# ------------------------------------------------------------------------------------------------ #
# controller listens from 2 topics for each controller
# 1) contoller name (eg. vive_left) gives the output of the buttons on each controller
# 2) `controller_name`_as_posestamped gives the location of the vive controller wrt the headset
# 2 types of messages: 1) geometry_msgs/PoseStamped 2) sensor_msgs/Joy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Joy
import intera_interface
from pid_controller import PIDControllerTorque
from tf.transformations import *

# ------------------------------------------------------------------------------------------------ #

class sawyerTeleoperation(object):
    def __init__(self):
        rospy.init_node('sawyerTeleoperation',anonymous=True)
        
        self.position = None
        self.left_button_state, self.right_button_state = None, None

        self.initControllerListener()
        self.tfListener = tf.TransformListener()
        self.limb = intera_interface.Limb('right')
        # self.gripper = intera_interface.Gripper('right')
        self.PD = PIDControllerTorque(kp=15, kd=6.5)

    def buttonsPressedLeft(self, data):
        self.left_button_state = data.buttons
        self.left_button_axes = data.axes
    
    def buttonsPressedRight(self, data):
        self.right_button_state = data.buttons
        self.right_button_axes = data.axes

    def initControllerListener(self):
        rospy.Subscriber("vive_left", Joy, self.buttonsPressedLeft)
        rospy.Subscriber("vive_right", Joy, self.buttonsPressedRight)

    def getControllerPositionWRTWorld(self, controller_name="left"):
        if controller_name == "left" : controller_name = '/left_controller'
        elif contoller_name == "right": controller_name = '/right_controller'
        else: raise Exception("either specify \"right\" or \"left\" for controller_name. param controller_name currently {}".format(controller_name)) 
        try:
            (trans,rot) = self.tfListener.lookupTransform('/base', controller_name, rospy.Time(0))
            # rot = Quaternion(rot[0], rot[1], rot[2], rot[3])
            return trans,rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    @property
    def robotPosition(self):
        try:
            (trans,rot) = self.tfListener.lookupTransform('/base', '/right_gripper_tip', rospy.Time(0))
            return np.asarray(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        

    @property
    def robotGripperOri(self):
        try:
            (trans,rot) = self.tfListener.lookupTransform('/base', '/right_gripper_tip', rospy.Time(0))
            return np.asarray(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        

    def convertMsgToJointAngles(self, msg):
        jointAngles = np.asarray([msg["right_j0"], msg["right_j1"], msg["right_j2"], msg["right_j3"], msg["right_j4"], msg["right_j5"], msg["right_j6"]])
        return jointAngles

    def gripperSM(self, gripperButton):
        if(gripperButton):
            if(not '''isgripping'''):
                # if the gripper is not in the gripper state, keep applying force
                pass
            else:
                # dont grip, because we're already gripping an object
                pass
        else:
            # open gripper
            pass

    def run(self):
        self.limb.move_to_neutral(speed=0.2)
        msg = "=========================starting========================="
        rospy.loginfo(msg)
        start_cont_pos = None
        self.r = rospy.Rate(500)
       
        displacement_coeff = 1.0
        held_ori = self.robotGripperOri
        while not rospy.is_shutdown():
            #print(self.robotGripperOri)
            
            if( (self.left_button_state is not None) and not(self.left_button_state[0])):
                start_cont_pos, start_cont_ori = self.getControllerPositionWRTWorld(controller_name="left")
                # inv_start_cont_ori = [start_cont_ori[0], start_cont_ori[1], start_cont_ori[2], -start_cont_ori[3]]
                start_robot_pos = self.robotPosition
                current_robot_ori = self.robotGripperOri
                if(self.left_button_state[3]):
                    self.limb.move_to_neutral(speed=0.2)
                    held_ori = self.robotGripperOri
                    

            if( (self.left_button_state is not None) and (self.left_button_state[0])):
                curr_cont_pos, curr_cont_ori  = self.getControllerPositionWRTWorld(controller_name="left")
                # current_robot_ori = self.robotGripperOri
                displacement = np.subtract(np.asarray(curr_cont_pos), np.asarray(start_cont_pos))
                updatedPosition = np.add(displacement_coeff * displacement, start_robot_pos)

                if(self.left_button_state[2]):
                    inv_cont_ori = [curr_cont_ori[0], curr_cont_ori[1], curr_cont_ori[2], -curr_cont_ori[3]]
                    # delta_orientation = tf.transformations.quaternion_multiply(curr_cont_ori, inv_start_cont_ori)
                    delta_orientation = tf.transformations.quaternion_multiply(start_cont_ori, inv_cont_ori)
                    #delta_orientation = tf.transformations.quaternion_multiply(inv_start_cont_ori, curr_cont_ori)
                    #delta_orientation[3] *= -1
                    print(delta_orientation)
                    current_ori = tf.transformations.quaternion_multiply(current_robot_ori, delta_orientation)
                    held_ori = current_ori
                    current_ori = Quaternion(current_ori[0], current_ori[1], current_ori[2], current_ori[3])
                else:
                    temp_ori = held_ori
                    current_ori = Quaternion(temp_ori[0], temp_ori[1], temp_ori[2], temp_ori[3])
                
                pose_msg = Pose()
                pose_msg.position = Point(updatedPosition[0], updatedPosition[1], updatedPosition[2])
                pose_msg.orientation = current_ori 
                final_joint_angles = self.limb.ik_request(pose_msg, "right_gripper_tip")
                if(type(final_joint_angles) is not bool):

                    initial_joint_angles = self.limb.joint_angles()

                    final_joint_angles = self.convertMsgToJointAngles(final_joint_angles)
                    initial_joint_angles = self.convertMsgToJointAngles(initial_joint_angles)

                    torques = self.PD.update(initial_joint_angles, final_joint_angles, self.convertMsgToJointAngles(self.limb.joint_velocities()))
                    
                    torques = {'right_j0': torques[0], 'right_j1': torques[1], 'right_j2': torques[2], 'right_j3': torques[3], \
                               'right_j4': torques[4], 'right_j5': torques[5], 'right_j6': 0}

                    self.limb.set_joint_torques(torques)

                    msg = "robotPosition:{}, updatedPosition:{}, torques:{}".format(self.robotPosition, updatedPosition, torques)
                    #rospy.loginfo(msg)
                
            # gripper logic
            if(self.right_button_state is not None):
                self.gripperSM(self.right_button_state[3])
                
                self.r.sleep()


if __name__ == "__main__":
    st = sawyerTeleoperation()
    st.run()

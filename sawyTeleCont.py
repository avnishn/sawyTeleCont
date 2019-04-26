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

# ------------------------------------------------------------------------------------------------ #

class sawyerTeleoperation(object):
    def __init__(self):
        rospy.init_node('sawyerTeleoperation',anonymous=True)
        
        self.position = None
        self.buttonsState = None

        self.initControllerListener()
        self.tfListener = tf.TransformListener()
        self.limb = intera_interface.Limb('right')
        self.initial_pos = self.limb.endpoint_pose()
        self.startingRobotPosition = self.limb.endpoint_pose()
        # change to torque pid controller
        self.PD = PIDControllerTorque(kp=15, kd=7)

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
        try:    
            (trans,rot) = self.tfListener.lookupTransform('/base', '/right_gripper_tip', rospy.Time(0))
            return np.asarray(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        

    @property
    def robotGripperOri(self):
        ori = self.limb.endpoint_pose()['orientation']
        return ori

    def convertMsgToJointAngles(self, msg):
        jointAngles = np.asarray([msg["right_j0"], msg["right_j1"], msg["right_j2"], msg["right_j3"], msg["right_j4"], msg["right_j5"], msg["right_j6"]])
        return jointAngles

    def run(self):
        # self.limb.set_joint_position_speed(speed=0.1)
        self.limb.move_to_neutral(speed=0.2)
        msg = "=========================starting========================="
        rospy.loginfo(msg)
        startControllerPosition = None
        self.r = rospy.Rate(500)
        initialOri = self.robotGripperOri
        displacement_coeff = 1.0
    
        while not rospy.is_shutdown():
            #self.limb.exit_control_mode()
            if( (self.buttonsState is not None) and not(self.buttonsState[0])):
                startControllerPosition, _ = self.getControllerPositionWRTWorld()
                startRobotPosition = self.robotPosition
                if(self.buttonsState[3]):
                    self.limb.move_to_neutral(speed=0.2)
                # self.limb.exit_control_mode()
                start_time = time.time()
                control_torque = {'right_j0': 0.0,'right_j1': 0.0,'right_j2': 0.0,'right_j3': 0.0,'right_j4': 0.0,'right_j5': 0.0, 'right_j6': 0.0}


            if( (self.buttonsState is not None) and (self.buttonsState[0])):
                currentControllerPosition, _ = self.getControllerPositionWRTWorld()

                # displacement = currControllerPos - startControllerPos
                # updated position = displacement + startingRobotPos
                # pointToMoveTo = pidcontroler(updatedPosition)
                # get joint angles using IK solver
                # send to robot
                
                # displacement = np.asarray([0.0, 0, 0.1])
                displacement = np.subtract(np.asarray(currentControllerPosition), np.asarray(startControllerPosition))


                # for x in displacement:
                #    x *= 0.5
                # updatedPosition = np.add(displacement_coeff * displacement, self.robotPosition)
                updatedPosition = np.add(displacement_coeff * displacement, startRobotPosition)

                # startControllerPosition = currentControllerPosition

                # pid_pos = self.PID.update(self.robotPosition, updatedPosition, rospy.get_time())

                # construct message
                pose_msg = Pose()
                pose_msg.position = Point(updatedPosition[0], updatedPosition[1], updatedPosition[2])
                pose_msg.orientation = initialOri #self.robotGripperOri
                final_joint_angles = self.limb.ik_request(pose_msg, "right_gripper_tip")
                if(type(final_joint_angles) is not bool):

                    initial_joint_angles = self.limb.joint_angles()
                    # print(final_joint_angles)
                    # print(initial_joint_angles)


                    final_joint_angles = self.convertMsgToJointAngles(final_joint_angles)
                    initial_joint_angles = self.convertMsgToJointAngles(initial_joint_angles)

                    torques = self.PD.update(initial_joint_angles, final_joint_angles, self.convertMsgToJointAngles(self.limb.joint_velocities()))
                    
                    torques = {'right_j0': torques[0], 'right_j1': torques[1], 'right_j2': torques[2], 'right_j3': torques[3], \
                               'right_j4': torques[4], 'right_j5': torques[5], 'right_j6': torques[6]}

                    # torques = {'right_j0': 0.0, 'right_j1': 0.0, 'right_j2': 0.0, 'right_j3': 0.0, 'right_j4': 0.0, 'right_j5': 0.0, 'right_j6': 0.0}
                    curr_time = time.time()
                    if(curr_time-start_time < 0.1):
                        control_torque = copy.deepcopy(torques)
                    else:
                        self.limb.set_joint_torques(torques)

                    msg = "robotPosition:{}, updatedPosition:{}, torques:{}".format(self.robotPosition, updatedPosition, torques)
                    #rospy.loginfo(msg)
                self.r.sleep()


if __name__ == "__main__":
    st = sawyerTeleoperation()
    st.run()

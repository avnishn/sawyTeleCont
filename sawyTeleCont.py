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
        self.PD = PIDControllerTorque(kp=2, kd=20)

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
        pos = self.limb.endpoint_pose()['position']
        return np.asarray([pos.x, pos.y, pos.z])

    @property
    def robotGripperOri(self):
        ori = self.limb.endpoint_pose()['orientation']
        return ori

    def convertMsgToJointAngles(self, msg):
        jointAngles = np.asarray(msg["right_j6"], msg["right_j5"], msg["right_j4"], msg["right_j3"], msg["right_j2"], msg["right_j1"], msg["right_j0"])

    def run(self):
        self.limb.set_joint_position_speed(speed=0.1)
        self.limb.move_to_neutral(speed=0.1)
        msg = "=========================starting========================="
        rospy.loginfo(msg)
        startControllerPosition = None
        self.r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.limb.exit_control_mode()
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
                    x *= 0.5
                updatedPosition = np.add(displacement, self.robotPosition)
                startControllerPosition = currentControllerPosition

                # pid_pos = self.PID.update(self.robotPosition, updatedPosition, rospy.get_time())

                # construct message
                pose_msg = Pose()
                pose_msg.position = Point(updatedPosition[0], updatedPosition[1], updatedPosition[2])
                pose_msg.orientation = self.robotGripperOri
                final_joint_angles = self.limb.ik_request(pose_msg, "right_gripper_tip")
                initial_joint_angles = self.limb.joint_angles()

                final_joint_angles = self.convertMsgToJointAngles(final_joint_angles)
                initial_joint_angles = self.convertMsgToJointAngles(initial_joint_angles)

                torques = self.PD.update(initial_joint_angles, final_joint_angles, rospy.get_time())
                # {'right_j6': -0.008129313418289359, 'right_j5': -0.20161485324696887, 'right_j4': 0.23198861560457232, 'right_j3': -1.2608710333774904, 'right_j2': -0.99524860
                # 44219659, 'right_j1': -5.53091341181409, 'right_j0': -3.086437940859388e-05}
                
                torques = {'right_j6': torques[0], 'right_j5': torques[1], 'right_j4': torques[2], 'right_j3': torques[3], 'right_j2': torques[4], 'right_j1': torques[5], 'right_j0': torques[6]}
                
                self.limb.set_joint_torques(torques)

                msg = "robotPosition:{}, torques:{}".format(self.robotPosition, torques)
                rospy.loginfo(msg)
                self.r.sleep()


if __name__ == "__main__":
    st = sawyerTeleoperation()
    st.run()

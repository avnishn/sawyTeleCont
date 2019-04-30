#!/usr/bin/env python
import sys
import rospy
import numpy as np
import tf
import enum

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Joy
import intera_interface
from pid_controller import PIDControllerTorque
from sawyer import Sawyer
from vive_subscriber import vive_subscriber
# ------------------------------------------------------------------------------------------------ #

class macros(enum.Enum):
    LEFT_CONTROLLER = True
    RIGHT_CONTROLLER = False


class sawyerTeleoperation(object):
    def __init__(self):
        rospy.init_node('sawyerTeleoperation',anonymous=True)

      
        self.limb = intera_interface.Limb('right')

        self.PD = PIDControllerTorque(kp=15, kd=7)

        self.sawyer = Sawyer(moveit_group = None, control_mode = 'torque')
        self.vive = Vive_Subscriber()


    def convertMsgToJointAngles(self, msg):
        jointAngles = np.asarray([msg["right_j0"], msg["right_j1"], msg["right_j2"], msg["right_j3"], msg["right_j4"], msg["right_j5"], msg["right_j6"]])
        return jointAngles

    def run(self):
        self.sawyer.move_to_neutral_position(0.2)
        msg = "=========================starting========================="
        rospy.loginfo(msg)
        startControllerPosition = None
        self.r = rospy.Rate(500)
        initialOri = self.sawyer.gripper_pose['orientation']
        displacement_coeff = 1.0

        while not rospy.is_shutdown():
            if(self.vive.left_buttons_state is not None):
                if(not(self.vive.left_buttons_state[0])):
                    startControllerPosition, _ = self.vive.getControllerPositionWRTWorld(macros.LEFT_CONTROLLER)
                    startRobotPosition = np.asarray(self.sawyer.gripper_pose['position'])
                    if(self.buttonsStateLeft[3]):
                        self.sawyer.move_to_neutral_position(0.2)

                else:
                    currentControllerPosition, _ = self.vive.getControllerPositionWRTWorld(macros.LEFT_CONTROLLER)
                    displacement = np.subtract(np.asarray(currentControllerPosition), np.asarray(startControllerPosition))
                    updatedPosition = np.add(displacement_coeff * displacement, startRobotPosition)
                    # construct message
                    pose_msg = Pose()
                    pose_msg.position = Point(updatedPosition[0], updatedPosition[1], updatedPosition[2])
                    pose_msg.orientation = initialOri
                    final_joint_angles = self.limb.ik_request(pose_msg, "right_gripper_tip")
                    if(type(final_joint_angles) is not bool):

                        initial_joint_angles = self.limb.joint_angles()
                        final_joint_angles = self.convertMsgToJointAngles(final_joint_angles)
                        initial_joint_angles = self.convertMsgToJointAngles(initial_joint_angles)
                        torques = self.PD.update(initial_joint_angles, final_joint_angles, self.convertMsgToJointAngles(self.limb.joint_velocities()))
                        torques = {'right_j0': torques[0], 'right_j1': torques[1], 'right_j2': torques[2], 'right_j3': torques[3], \
                                'right_j4': torques[4], 'right_j5': torques[5], 'right_j6': torques[6]}
                        self.sawyer.send_command(torques)

                        msg = "robotPosition:{}, updatedPosition:{}, torques:{}".format(self.sawyer.gripper_pose['position]'], updatedPosition, torques)
                        rospy.loginfo(msg)
                    self.r.sleep()


if __name__ == "__main__":
    st = sawyerTeleoperation()
    st.run()

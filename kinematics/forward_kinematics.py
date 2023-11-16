import os
import sys
import numpy as np
from numpy.matlib import matrix, identity
from joint_control.recognize_posture import PostureRecognitionAgent
from joint_control.angle_interpolation import AngleInterpolationAgent

# Add PYTHONPATH
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost', simspark_port=3100, teamname='DAInamite', player_id=0, sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {joint: identity(4) for joint in self.joint_names}

        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
        }

        self.jLength = {
            'Head': [[0, 0, 126.5], [0, 0, 0]],
            'LArm': [[0, 98, 100], [0, 0, 0], [105, 15, 0], [0, 0, 0]],
            'LLeg': [[0, 50, -85], [0, 0, 0], [0, 0, 0], [0, 0, -100], [0, 0, -102.9], [0, 0, 0]],
            'RLeg': [[0, -50, -85], [0, 0, 0], [0, 0, 0], [0, 0, -100], [0, 0, -102.9], [0, 0, 0]],
            'RArm': [[0, -98, 100], [0, 0, 0], [105, 15, 0], [0, 0, 0]]
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        T = identity(4)

        if joint_name in ["HeadYaw", "LElbowYaw", "RElbowYaw", "LHipYawPitch", "RHipYawPitch"]:
            rotation_matrix = np.array([[np.cos(joint_angle), -np.sin(joint_angle), 0, 0],
                                       [np.sin(joint_angle), np.cos(joint_angle), 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]])
        elif joint_name.endswith("Pitch"):
            rotation_matrix = np.array([[np.cos(joint_angle), 0, np.sin(joint_angle), 0],
                                       [0, 1, 0, 0],
                                       [-np.sin(joint_angle), 0, np.cos(joint_angle), 0],
                                       [0, 0, 0, 1]])
        elif joint_name.endswith("Roll"):
            rotation_matrix = np.array([[1, 0, 0, 0],
                                       [0, np.cos(joint_angle), -np.sin(joint_angle), 0],
                                       [0, np.sin(joint_angle), np.cos(joint_angle), 0],
                                       [0, 0, 0, 1]])

        T = np.dot(T, rotation_matrix)

        for chain_name, chain_joints in self.chains.items():
            if joint_name in chain_joints:
                idx = chain_joints.index(joint_name)
                translation_vector = self.jLength[chain_name][idx]
                T[0, 3] = translation_vector[0]
                T[1, 3] = translation_vector[1]
                T[2, 3] = translation_vector[2]

        return T

    def forward_kinematics(self, joints):
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                T_local = self.local_trans(joint, angle)
                T = np.dot(T, T_local)
                self.transforms[joint] = T


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()

'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
from scipy.spatial.transform import Rotation as R

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        }

        self.joint_length = {
            'HeadYaw': (0, 0, 126.5),
            'HeadPitch': (0, 0, 0),

            # Arms
            'LShoulderPitch': (0, 98, 100),
            'LShoulderRoll': (0, 0, 0),
            'LElbowYaw': (105, 15, 0),
            'LElbowRoll': (0, 0, 0),
            'LWristYaw': (55.95, 0, 0),
            # Y gespiegelt
            'RShoulderPitch': (0, -98, 100),
            'RShoulderRoll': (0, 0, 0),
            'RElbowYaw': (105, 15, 0),
            'RElbowRoll': (0, 0, 0),
            'RWristYaw': (55.95, 0, 0),

            # Legs
            'LHipYawPitch': (0, 50, -85),
            'LHipRoll': (0, 0, 0),
            'LHipPitch': (0, 0, 0),
            'LKneePitch': (0, 0, -100),
            'LAnklePitch': (0, 0, -102.90),
            'LAnkleRoll': (0, 0, 0),
            # Y gespiegelt
            'RHipYawPitch': (0, -50, -85),
            'RHipRoll': (0, 0, 0),
            'RHipPitch': (0, 0, 0),
            'RKneePitch': (0, 0, -100),
            'RAnklePitch': (0, 0, -102.90),
            'RAnkleRoll': (0, 0, 0)
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        # print(np.round(self.from_trans(self.transforms["RElbowRoll"]), 4))
        return super(ForwardKinematicsAgent, self).think(perception)

    def rotation_x(self, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        return np.array([
            [1, 0,  0, 0],
            [0, c, -s, 0],
            [0, s,  c, 0],
            [0, 0,  0, 1]
        ])

    def rotation_y(self, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        return np.array([
            [ c, 0, s, 0],
            [ 0, 1, 0, 0],
            [-s, 0, c, 0],
            [ 0, 0, 0, 1]
        ])

    def rotation_z(self, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        return np.array([
            [ c, s, 0, 0],
            [-s, c, 0, 0],
            [ 0, 0, 1, 0],
            [ 0, 0, 0, 1]
        ])
    
    def translation(self, coords):
        x, y, z = coords
        ret = np.eye(4)
        ret[:3, 3] = np.array([x, y, z]) / 1000 # Conversion from mm in m
        return ret


    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = np.eye(4)
        # YOUR CODE HERE
        if 'YawPitch' in joint_name:
            T = T @ self.rotation_x(np.deg2rad(45))
            T = T @ self.rotation_z(joint_angle)
        elif 'Yaw' in joint_name:
            T = T @ self.rotation_z(joint_angle)
        elif 'Pitch' in joint_name:
            T = T @ self.rotation_y(joint_angle)
        elif 'Roll' in joint_name:
            T = T @ self.rotation_x(joint_angle)
        
        T = T @ self.translation(self.joint_length[joint_name])

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = np.eye(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl

                self.transforms[joint] = T
        return T

    def from_trans(self, t, column_trans = True):
        x, y, z = t[:3, 3]
        if not column_trans:
            x, y, z = t[3, :3]

        r_x, r_y, r_z = R.from_matrix(t[:3, :3]).as_euler("xyz", degrees=False)

        return np.array([x, y, z, r_x, r_y, r_z])

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()

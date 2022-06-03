'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fmin


"""

def error_func(theta, target):
    Ts = forward_kinematics(T0, l, theta)
    Te = matrix([from_trans(Ts[-1])]).T
    e = target - Te
    return linalg.norm(e)

theta = random.random(N)
def inverse_kinematics(x_e, y_e, theta_e, theta):
    target = matrix([[x_e, y_e, theta_e]]).T
    func = lambda t: error_func(t, target)
    return fmin(func, theta)

T = forward_kinematics(T0, l, theta)
show_robot_arm(T)
Te = matrix([from_trans(T[-1])])
"""

JOINT_CMD_NAMES = {'HeadYaw': "he1",
                   'HeadPitch': "he2",
                   'LShoulderPitch': "lae1",
                   'LShoulderRoll': "lae2",
                   'LElbowYaw': "lae3",
                   'LElbowRoll': "lae4",
                   'LHipYawPitch': "lle1",
                   'LHipRoll': "lle2",
                   'LHipPitch': "lle3",
                   'LKneePitch': "lle4",
                   'LAnklePitch': "lle5",
                   'LAnkleRoll': "lle6",
                   'RShoulderPitch': "rae1",
                   'RShoulderRoll': "rae2",
                   'RElbowYaw': "rae3",
                   'RElbowRoll': "rae4",
                   'RHipYawPitch': "rle1",
                   'RHipRoll': "rle2",
                   'RHipPitch': "rle3",
                   'RKneePitch': "rle4",
                   'RAnklePitch': "rle5",
                   'RAnkleRoll': "rle6"}

JOINT_NAMES = JOINT_CMD_NAMES.keys()

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def error_func(self, theta, target, effector_name):
        self.forward_kinematics(theta)
        j_name = self.chains[effector_name][-1]
        T = self.transforms[j_name]

        Te = self.from_trans(np.array(T))
        Tt = self.from_trans(np.array(target), column_trans = False)

        err = np.linalg.norm(Tt - Te)
        # print(np.round(Te, 3), Tt, 3)
        return err

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        # YOUR CODE HERE

        chain = self.chains[effector_name]
        chain_angles = np.zeros(len(chain))
        print("Chain", chain)

        base_dict = dict(zip(JOINT_NAMES, np.zeros(len(JOINT_NAMES))))

        def func(t): 
            base_dict.update(dict(zip(chain, t)))
            return self.error_func(base_dict, transform, effector_name)

        joint_angles = fmin(func, chain_angles)
        # print("Foo", joint_angles)

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        chain = self.chains[effector_name]
        joint_angles = self.inverse_kinematics(effector_name, transform)

        self.keyframes = ([], [], [])

        for name, angle in zip(chain, joint_angles):
            # print(name, ": ", angle)
            self.keyframes[0].append(name)
            self.keyframes[1].append([2])
            self.keyframes[2].append([
                [angle, [0, 0, 0], [0, 0, 0]],
            ])

        self.is_init = True
    

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()

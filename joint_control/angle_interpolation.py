'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from cmath import inf
from pid import PIDAgent
from keyframes import hello, wipe_forehead, leftBackToStand
from scipy import interpolate
import numpy as np
import numpy.linalg as lg

def interpolate_o(x, y):
    n = x.shape[0]

    X = np.zeros((4 * n - 4, 4 * n - 4))
    Y = np.zeros(4 * n - 4)

    for i in range(n - 1):
        x_s = x[i]
        x_e = x[i+1]

        # Wert Bedingung
        X[4*i, 4*i:4*(i+1)]     = np.array([1, x_s, x_s ** 2, x_s ** 3])
        X[4*i + 1, 4*i:4*(i+1)] = np.array([1, x_e, x_e ** 2, x_e ** 3])

        Y[4*i] = y[i]
        Y[4*i + 1] = y[i+1]

        if i != n - 2:
            # Steigung und Kurven Bedingung
            X[4*i + 2, 4*i:4*(i+1)] = np.array([0, 1, 2 * x_e, 3 * x_e ** 2])
            X[4*i + 3, 4*i:4*(i+1)] = np.array([0, 0, 2, 6 * x_e])

            # Steigung und Kurven Bedingung an nächsten Abschnitt
            X[4*i + 2, 4*(i+1):4*(i+2)] = np.array([0, -1, -2 * x_e, -3 * x_e ** 2])
            X[4*i + 3, 4*(i+1):4*(i+2)] = np.array([0, 0, -2, -6 * x_e])
    
    X[4*i + 2, 0:4] = np.array([0, 0, 2, 6 * x[0]])
    X[4*i + 3, -4:] = np.array([0, 0, 2, 6 * x[-1]])

    coeffs = lg.solve(X, Y)

    def fun(x_value):
        for i in range(n - 1):
            if x[i] <= x_value <= x[i+1]:
                s_i = 4 * i
                return coeffs[s_i] + coeffs[s_i + 1] * x_value + coeffs[s_i + 2] * x_value**2 + coeffs[s_i + 3] * x_value**3
        
        # Return last value of interpolation
        return coeffs[-4] + coeffs[-3] * x[-1] + coeffs[-2] * x[-1]**2 + coeffs[-1] * x[-1]**3
    
    return fun


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

        self.is_init = True
        self.motion_finished = True
        self.t_start = 0
        self.interpolation_funcs = {}
        self.interpolation_max = -inf

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)
    
    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        names, times, values = keyframes

        if self.is_init:
            print("Initialising Motion")
            self.t_start = perception.time
            self.is_init = False
            self.interpolation_max = -inf
            self.motion_finished = False

            for j_idx, joint in enumerate(names):
                
                if joint not in perception.joint:
                    continue

                j_times = np.concatenate([[0], times[j_idx]]) # Get Time
                j_values = np.concatenate([[perception.joint[joint]], list(map(lambda x: x[0], values[j_idx]))]) # Get Radiant

                self.interpolation_funcs[joint] = interpolate_o(j_times, j_values)
                self.interpolation_max = max(self.interpolation_max, max(j_times))

        t = perception.time - self.t_start

        if t > self.interpolation_max:
            self.motion_finished = True

        for j_idx, joint in enumerate(names):
            if joint not in perception.joint:
                continue

            target_joints[joint] = self.interpolation_funcs[joint](t)

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    # agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

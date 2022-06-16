'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle

"""
'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'AngleX', 'AngleY'], where 'AngleX' and 'AngleY' are body angle (e.g. ```Perception.imu```)
"""

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.prev_post = 'unknown'
        self.posture = 'unknown'

        # ROBOT_POSE_CLF = 'robot_pose.pkl'
        # self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF, "rb"))
        self.classes = ['Knee', 'Stand', 'HeadBack', 'Frog', 'Crouch', 'Sit', 'Left', 'Back', 'Right', 'StandInit', 'Belly']

    def think(self, perception):
        self.prev_post = self.posture
        self.posture = self.recognize_posture(perception)

        if self.prev_post != self.posture:
            print(self.posture)

        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        features = []

        for c in ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']:
            features.append(perception.joint[c])

        features.append(perception.imu[0])
        features.append(perception.imu[1])

        # posture_id = self.posture_classifier.predict([features])[0]

        return self.classes[0]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

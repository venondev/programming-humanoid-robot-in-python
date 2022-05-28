'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes import hello, wipe_forehead, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand


class StandingUpAgent(PostureRecognitionAgent):

    def __init__(self, simspark_ip='localhost', simspark_port=3100, teamname='DAInamite', player_id=0, sync_mode=True):
        super().__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)

        self.standup_motions = {
            "Back": leftBackToStand,
            "Belly": leftBellyToStand,
        }


    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture

        if posture in self.standup_motions and self.motion_finished:
            self.keyframes = self.standup_motions[posture]()
            self.is_init = True


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 15  # in seconds
        self.stiffness_off_cycle = 3  # in seconds
        self.cur_state = "on"

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints

            if self.cur_state == "on":
                print("Turning off Joints")
                self.cur_state = "off"
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints

            if self.cur_state == "off":
                print("Turning on Joints")
                self.cur_state = "on"
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()

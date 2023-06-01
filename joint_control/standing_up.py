'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes import (
    hello,
    leftBackToStand,
    leftBellyToStand,
    rightBackToStand,
    rightBellyToStand,
    wipe_forehead,
)


class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up(perception)
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self, perception):
        # Initially do not start standing up
        if perception.time - self.time <= 5:
            return
        posture = self.posture
        hasKeyframe = self.keyframes != None
        if hasKeyframe:
            return
        if posture == 'Back':
            self.addKeyframe(leftBackToStand())
            return
        if posture == 'Belly':
            self.addKeyframe(leftBellyToStand())
            return


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles'''

    def __init__(
        self,
        simspark_ip='172.29.32.1',
        simspark_port=3100,
        teamname='DAInamite',
        player_id=0,
        sync_mode=True,
    ):
        super(TestStandingUpAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode
        )
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 15  # in seconds
        self.stiffness_off_cycle = 2  # in seconds

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if (
            time_now - self.stiffness_on_off_time
            > self.stiffness_on_cycle + self.stiffness_off_cycle
        ):
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()

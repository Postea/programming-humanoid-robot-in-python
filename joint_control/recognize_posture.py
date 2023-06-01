'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import (
    hello,
    leftBackToStand,
    leftBellyToStand,
    rightBackToStand,
    rightBellyToStand,
    wipe_forehead,
)
import pickle

ROBOT_POSE_CLF = 'robot_pose.pkl'
ROBOT_POSTURE_MAP = {
    0: 'Back',
    1: 'Belly',
    2: 'Crouch',
    3: 'Frog',
    4: 'HeadBack',
    5: 'Knee',
    6: 'Left',
    7: 'Right',
    8: 'Sit',
    9: 'Stand',
    10: 'StandInit',
}
ROBOT_JOINT_DATA_ORDER = [
    'LHipYawPitch',
    'LHipRoll',
    'LHipPitch',
    'LKneePitch',
    'RHipYawPitch',
    'RHipRoll',
    'RHipPitch',
    'RKneePitch',
]


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(
        self,
        simspark_ip='172.27.192.1',
        simspark_port=3100,
        teamname='DAInamite',
        player_id=0,
        sync_mode=True,
    ):
        super(PostureRecognitionAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode
        )
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(
            open(ROBOT_POSE_CLF, 'rb')
        )  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        # print(self.posture)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # Data for the prediction
        data = [perception.joint[j] for j in ROBOT_JOINT_DATA_ORDER]
        data.append(perception.imu[0])
        data.append(perception.imu[1])
        # Predict the posture
        posture = ROBOT_POSTURE_MAP[self.posture_classifier.predict([data])[0]]
        return posture


if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

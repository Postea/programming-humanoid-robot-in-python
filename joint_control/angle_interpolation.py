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


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(
        self,
        simspark_ip='172.27.192.1',
        simspark_port=3100,
        teamname='DAInamite',
        player_id=0,
        sync_mode=True,
    ):
        super(AngleInterpolationAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode
        )
        self.keyframes = ([], [], [])
        self.keyframe_maxtime = 0
        self.time = self.perception.time

    def addKeyframe(self, keyframes):
        self.keyframes = keyframes
        self.keyframe_maxtime = 0
        self.time = self.perception.time

    def think(self, perception):
        if self.keyframes:
            target_joints = self.angle_interpolation(self.keyframes, perception)
            target_joints['RHipYawPitch'] = target_joints[
                'LHipYawPitch'
            ]  # copy missing joint in keyframes
            self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        names, times, keys = keyframes

        t = perception.time - self.time

        # Calculate the maxtime if it hasnt been calculated yet
        if self.keyframe_maxtime == 0:
            for tt in times:
                self.keyframe_maxtime = max(self.keyframe_maxtime, max(tt))

        # Eject keyframe when maxtime is reached
        if t > self.keyframe_maxtime:
            self.keyframes = None
            return self.target_joints

        target_joints = {}

        # Help function
        def pointInLine(a, b, t):
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

        # Linear:
        def linear(joint_index):
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]
            # Get the correct partial function
            # (i-1, i) becomes the index of the partial function
            i = 0
            if t < joint_times[-1]:
                while t > joint_times[i]:
                    i = i + 1
            # Normalize the times
            starttime = joint_times[i - 1]
            endtime = joint_times[i]
            normalized_t = (t - starttime) / (endtime - starttime)
            # The actual function
            startkey = joint_keys[i - 1][0]
            endkey = joint_keys[i][0]

            p0 = (starttime, startkey)
            p1 = (endtime, endkey)
            value = pointInLine(p0, p1, normalized_t)[1]

            return value

        # Cubic Bezier:
        def bezier(joint_index):
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]
            # Get the correct partial function
            # (i-1, i) becomes the index of the partial function
            i = 0
            if t < joint_times[-1]:
                while t > joint_times[i]:
                    i = i + 1
            # Normalize the times
            starttime = joint_times[i - 1]
            endtime = joint_times[i]
            normalized_t = (t - starttime) / (endtime - starttime)
            # The actual function
            startkey = joint_keys[i - 1][0]
            endkey = joint_keys[i][0]

            p0 = (starttime, startkey)
            p1 = (endtime, endkey)
            h1 = (
                starttime + joint_keys[i - 1][1][1],
                startkey + joint_keys[i - 1][1][2],
            )
            h2 = (
                starttime + joint_keys[i - 1][2][1],
                startkey + joint_keys[i - 1][2][2],
            )

            a = pointInLine(p0, h1, normalized_t)
            b = pointInLine(h1, h2, normalized_t)
            c = pointInLine(h2, p1, normalized_t)

            d = pointInLine(a, b, normalized_t)
            e = pointInLine(b, c, normalized_t)

            value = pointInLine(d, e, normalized_t)[1]
            return value

        target_joints = self.target_joints
        for joint_index in range(0, len(names)):
            target_joints[names[joint_index]] = linear(joint_index)
        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

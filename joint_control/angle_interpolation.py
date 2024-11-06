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
from keyframes import hello, wipe_forehead, rightBellyToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        print("Target joints:", target_joints)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        
        #here
        names, times, keys = keyframes
        currentTime = perception.time 
        
        for i, jointName in enumerate(names):
            jointTime = times[i]
            jointKey = keys[i]
            updatedCurrentTime = currentTime % max(jointTime)
            
            for j in range(1, len(jointTime)):
                if updatedCurrentTime < jointTime[j]:
                    t = (updatedCurrentTime - jointTime[j-1]) / (jointTime[j] - jointTime[j-1])
                    
                    p0 = jointKey[j-1][0]
                    p1 = p0 + jointKey[j-1][1][2] * (jointTime[j] - jointTime[j-1]) / 3
                    p2 = jointKey[j][0] + jointKey[j][2][2] * (jointTime[j] - jointTime[j-1]) / 3
                    p3 = jointKey[j][0]

                    target_joints[jointName] = (1-t)**3*p0+3*(1-t)**2*t*p1+3*(1-t)*t**2*p2+t**3*p3
                    break
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

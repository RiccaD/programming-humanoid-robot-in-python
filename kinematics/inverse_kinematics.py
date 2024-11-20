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
from numpy.matlib import identity, arctan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        print(effector_name)
        legChains = {
            "LLeg": [
                {"name": "lHipRoll", "axis": "X", "minAngle": -0.379472, "maxAngle": 0.790477, "link": {"x": 0.0, "y": 0.05, "z": -0.085}},
                {"name": "lHipPitch", "axis": "Y", "minAngle": -1.535889, "maxAngle": 0.484090, "link": {"x": 0.0, "y": 0.0, "z": 0.0}},
                {"name": "lKneePitch", "axis": "Y", "minAngle": -0.092346, "maxAngle": 2.112528, "link": {"x": 0.0, "y": 0.0, "z": -0.1}},
                {"name": "lAnklePitch", "axis": "Y", "minAngle": -1.189516, "maxAngle": 0.922747, "link": {"x": 0.0, "y": 0.0, "z": -0.1029}},
                {"name": "lAnkleRoll", "axis": "X", "minAngle": -0.397880, "maxAngle": 0.769001, "link": {"x": 0.0, "y": 0.0, "z": 0.0}}
            ],
            "RLeg": [
                {"name": "rHipRoll", "axis": "X", "minAngle": -0.790477, "maxAngle": 0.379472, "link": {"x": 0.0, "y": -0.05, "z": -0.085}},
                {"name": "rHipPitch", "axis": "Y", "minAngle": -1.535889, "maxAngle": 0.484090, "link": {"x": 0.0, "y": 0.0, "z": 0.0}},
                {"name": "rKneePitch", "axis": "Y", "minAngle": -0.103083, "maxAngle": 2.120198, "link": {"x": 0.0, "y": 0.0, "z": -0.1}},
                {"name": "rAnklePitch", "axis": "Y", "minAngle": -1.186448, "maxAngle": 0.932056, "link": {"x": 0.0, "y": 0.0, "z": -0.1029}},
                {"name": "rAnkleRoll", "axis": "X", "minAngle": -0.768992, "maxAngle": 0.397935, "link": {"x": 0.0, "y": 0.0, "z": 0.0}}
            ]
        }
        
        for joint in legChains[effector_name]:
            
            axis = joint["axis"]

            if axis == "X":
                angle = arctan2(transform[1, 3], transform[2, 3])
            elif axis == "Y":
                angle = arctan2(-transform[2, 3], transform[0, 3])
            elif axis == "Z":
                angle = arctan2(transform[0, 1], transform[0, 0])

            angle = max(joint["minAngle"], min(angle, joint["maxAngle"]))

            joint_angles.append(angle)
            
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        joint_angles = self.inverse_kinematics(effector_name, transform)
        self.keyframes = ([joint_angles], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()

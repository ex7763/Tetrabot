import numpy as np
import pybullet as p
import pybullet_data

import gym

import time
import os

client = p.connect(p.GUI)
p.setGravity(0, 0, -9.8, physicsClientId=client)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf",
                   flags=p.URDF_USE_SELF_COLLISION)

path = os.path.dirname(__file__)
robot = p.loadURDF( path + "/../tetrabot/urdf/tetrabot.urdf",
                   basePosition=[0,0,0.2],
                   flags=p.URDF_USE_SELF_COLLISION)
pos, orient = p.getBasePositionAndOrientation(robot)


print(pos, orient)


number_of_joints = p.getNumJoints(robot)

joint_dict = dict()
debug_param_list = []
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robot, joint_number)
    name = info[1].decode('utf-8')
    print(info)
    print(info[0], ": ", name)
    joint_dict[name] = info[0]
    debug_param_list.append(p.addUserDebugParameter(name, -90, 90, 0))

    p.changeDynamics(robot, info[0],
                     lateralFriction = 200000,
                     )
    #print(info)

def resetRobot():
    p.resetBasePositionAndOrientation(robot,
                                      [0, 0, 0.1],
                                      p.getQuaternionFromEuler([np.pi, np.pi, np.pi]))

    for key in joint_dict.keys():
        p.resetJointState(robot, # body unique id as returned by loadURDF etc
                          joint_dict[key], # link index in range [0..getNumJoints(bodyUniqueId)]
                          targetValue = 0. # the joint position (angle)
                          )


p.changeDynamics(plane, -1,
                 lateralFriction = 10000000,
                 )

class tetra(gym.Env):
    def __init__(self):
        action_low = - np.ones([16], dtype=np.float32)
        action_high = np.ones([16], dtype=np.float32)

        self.action_space = gym.spaces.Box(low=action_low, high=action_high, dtype=np.float32)

    def reset(self):
        resetRobot()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)

        action = action * 90. # -1. ~ 1. -> -90. ~ 90.(degree)

        joint_id = 0
        for key in joint_dict.keys():
            value = action[joint_id]
            p.setJointMotorControl2(robot,
                                joint_dict[key],
                                p.POSITION_CONTROL,
                                targetVelocity=value)

    def test(self):
        while True:
            pos, orient = p.getBasePositionAndOrientation(robot)
            #p.applyExternalForce(robot, 0, [5, 0, 0], pos, p.WORLD_FRAME)

            for key in joint_dict.keys():
                value = p.readUserDebugParameter(debug_param_list[joint_dict[key]])
                p.setJointMotorControl2(robot,
                                    joint_dict[key],
                                   # p.VELOCITY_CONTROL,
                                    p.POSITION_CONTROL,
                                    targetVelocity=value)
            p.stepSimulation()



if __name__ == '__main__':
    while True:
        time.sleep(0.01)
        p.stepSimulation()
    env = tetra()
    env.reset()
    action = np.ones([16]) * 0.3
    while(True):
        env.step(action)
        p.stepSimulation()
        time.sleep(0.01)

#env.test()


## Referene: https://www.programcreek.com/python/?code=Healthcare-Robotics%2Fassistive-gym%2Fassistive-gym-master%2Fassistive_gym%2Fenvs%2Fworld_creation.py
def enforce_joint_limits(self, body):
    # Enforce joint limits
    joint_states = p.getJointStates(body, jointIndices=list(range(p.getNumJoints(body, physicsClientId=self.id))), physicsClientId=self.id)
    joint_positions = np.array([x[0] for x in joint_states])
    lower_limits = []
    upper_limits = []
    for j in range(p.getNumJoints(body, physicsClientId=self.id)):
        joint_info = p.getJointInfo(body, j, physicsClientId=self.id)
        joint_name = joint_info[1]
        joint_pos = joint_positions[j]
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        if lower_limit == 0 and upper_limit == -1:
            lower_limit = -1e10
            upper_limit = 1e10
        lower_limits.append(lower_limit)
        upper_limits.append(upper_limit)
        # print(joint_name, joint_pos, lower_limit, upper_limit)
        if joint_pos < lower_limit:
            p.resetJointState(body, jointIndex=j, targetValue=lower_limit, targetVelocity=0, physicsClientId=self.id)
        elif joint_pos > upper_limit:
            p.resetJointState(body, jointIndex=j, targetValue=upper_limit, targetVelocity=0, physicsClientId=self.id)
    lower_limits = np.array(lower_limits)
    upper_limits = np.array(upper_limits)
    return lower_limits, upper_limits

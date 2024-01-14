import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import random


if __name__ == '__main__':
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

    p.resetSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
    urdfRootPath=pybullet_data.getDataPath()
    #print(urdfRootPath)
    p.setGravity(0,0,-10)

    planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])

    rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
    pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
    for i in range(7):
        p.resetJointState(self.pandaUid,i, rest_poses[i])
    p.resetJointState(self.pandaUid, 9, 0.08)
    p.resetJointState(self.pandaUid,10, 0.08)
    

    tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.975], globalScaling=1.5)

    tray1Uid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5,0.5,0], globalScaling=0.8)
    tray2Uid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5,-0.5,0], globalScaling=0.8)


    state_object = [0.43, 0.52, 0.05]
    #state_object= [random.uniform(0.5,0.8),random.uniform(-0.2,0.2),0.05]
    #self.objectUid = p.loadURDF(os.path.join(urdfRootPath, ""), basePosition=state_object, globalScaling=0.048)

    objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=state_object, globalScaling=1)
    state_robot = p.getLinkState(pandaUid, 11)[0]
    state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
    self.observation = state_robot + state_fingers
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
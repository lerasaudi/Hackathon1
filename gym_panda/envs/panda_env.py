import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import random


MAX_EPISODE_LEN = 20*100

class PandaEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        

        p.connect(p.GUI)
        

        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0.55, 0 , 0.5])
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        self.action_space = spaces.Box(np.array([-1]*4), np.array([1]*4))
        self.observation_space = spaces.Box(np.array([-1]*5), np.array([1]*5))

    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        
        orientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])
        dv = 0.00416
        #dv = 1
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv
        fingers = action[3]

        currentPose = p.getLinkState(self.pandaUid, 11)
        currentPosition = currentPose[0]
        newPosition = [currentPosition[0] + dx,
                       currentPosition[1] + dy,
                       currentPosition[2] + dz]
        jointPoses = p.calculateInverseKinematics(self.pandaUid,11,newPosition, orientation)[0:7]

        p.setJointMotorControlArray(self.pandaUid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers])

        p.stepSimulation()

        state_object, _ = p.getBasePositionAndOrientation(self.objectUid)
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])



        if -0.3>state_object[2]>-0.5:
            reward = 1
            done = True
        else:
            reward = 0
            done = False

        self.step_counter += 1

        if self.step_counter > MAX_EPISODE_LEN:
            reward = 0
            done = True

        info = {'object_position': state_object}
        self.observation = state_robot + state_fingers

        
        return np.array(self.observation).astype(np.float32), reward, done, info

    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.setTimeStep(0.5 / 240.0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        urdfRootPath=pybullet_data.getDataPath()
        #print(urdfRootPath)
        p.setGravity(0,0,-10)

        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])

        rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
        self.pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
        for i in range(7):
            p.resetJointState(self.pandaUid,i, rest_poses[i])
        p.resetJointState(self.pandaUid, 9, 0.08)
        p.resetJointState(self.pandaUid,10, 0.08)

        tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.95], globalScaling=1.5)

        tray1Uid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5,0.5,0], globalScaling=0.8)
        tray2Uid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5,-0.5,0], globalScaling=0.8)

        tray3Uid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5, 0, -0.1], globalScaling=0.7)

        state_object = [0.43, 0.52, 0.05]
        #state_object= [random.uniform(0.5,0.8),random.uniform(-0.2,0.2),0.05]
        
        self.objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=state_object, globalScaling=1)
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        self.observation = state_robot + state_fingers
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

        """
        arrow_start = np.array([0.27, 0.26, 0.1])
        arrow_end = arrow_start + np.array([0.27, 0.26, 0.31])

        p.addUserDebugLine(arrow_start, arrow_end, lineColorRGB=[1, 0, 0], lineWidth=0.1)
        """
        return np.array(self.observation).astype(np.float32)

    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]

        return rgb_array

    def _get_state(self):
        return self.observation
    
    def get_object_loc(self):
        position, orientation = p.getBasePositionAndOrientation(self.objectUid)
        return position
    
    def get_link_pose(self):
        _, position, orientation, _, _, _ = p.getLinkState(self.pandaUid, 6, computeLinkVelocity=0, computeForwardKinematics=0)
        return position, orientation

    def diplay_text1(self, stage):
 
        text_position = [-0.5, 0.25, 1]  # Specify the position in 3D space
        text_color = [1, 0, 0]  # Specify the color (RGB values)
        text_size = 2  # Specify the size of the text
        self.text = "Stage: {}".format(stage)

        text_id = p.addUserDebugText(
            self.text, 
            text_position, 
            textColorRGB=text_color, 
            textSize=text_size
            )
        
    def diplay_text2(self, stage):
 
        text_position = [-0.5, 0.25, 1]  # Specify the position in 3D space
        text_color = [1, 0, 0]  # Specify the color (RGB values)
        text_size = 2  # Specify the size of the text
        text = "Stage: {}".format(stage)

        text_id = p.addUserDebugText(
            text, 
            text_position, 
            textColorRGB=text_color, 
            textSize=text_size
            )
        

    def close(self):
        p.disconnect()

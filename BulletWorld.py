# -*-coding:Latin-1 -*
import pybullet as p
import logging
from CameraThread import *

import time
import threading
import utils_collision

logging.basicConfig(level=logging.DEBUG)


class BulletWorld:

    def __init__(self,
                 group=None,
                 target=None,
                 name=None,
                 args=(),
                 kwargs=None,
                 verbose=None):
        self.target = target
        self.name = name
        self.physicsClient = p.connect(p.GUI)  # Connect the pybullet module to the physics simulation
        self.p = p
        p.setGravity(0, 0, 0)

        # LOAD URDF for the different components
        self.ppsId = p.loadURDF("resources/urdf/pps.urdf") #ppsId contient en fait le bodyUniqueId
        self.myObstacle = p.loadURDF("resources/urdf/urdf_Obstacle.urdf")
        self.state_camera = {'dist': 3.5, 'yaw':13, 'pitch': -34, 'targetPos': [0,-1.95, -0.9]}
        self.camera_thread = CameraThread(self.state_camera)
        self.camera_thread.start()
        return


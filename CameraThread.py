# -*-coding:Latin-1 -*
import pybullet as p
import logging
import time
import threading
from typing import List
import utils_collision

logging.basicConfig(level=logging.DEBUG)


class CameraThread(threading.Thread):

    def __init__(self, state_camera,
                 group=None,
                 target=None,
                 name=None,
                 args=(),
                 kwargs=None,
                 verbose=None):
        super(CameraThread, self).__init__()  # Constructor of the super class Thread
        self.target = target
        self.name = name
        self.dist = p.addUserDebugParameter("Distance", 0, 2500, state_camera['dist'])
        self.yaw = p.addUserDebugParameter("Yaw", -180, 180, state_camera['yaw'])
        self.pitch = p.addUserDebugParameter("Pitch", -180, 180, state_camera['pitch'])
        self.target_pos = state_camera['targetPos']

        return

    def run(self):
        while p.isConnected():
            self.set_camera()
            time.sleep(0.4)
        return

    def set_camera(self):
        dist = p.readUserDebugParameter(self.dist)
        yaw = p.readUserDebugParameter(self.yaw)
        pitch = p.readUserDebugParameter(self.pitch)
        p.resetDebugVisualizerCamera(dist, yaw, pitch, self.target_pos)
        return
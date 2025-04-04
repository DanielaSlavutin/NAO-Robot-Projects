import numpy as np
from naoqi import ALBroker, ALProxy
import cv2
import time
import math

class NAO:
    def __init__(self, IP="127.0.0.1", PORT=9559):
        self.myBroker = ALBroker("myBroker", "0.0.0.0", 0, IP, PORT)
        self.motion = ALProxy("ALMotion", IP, PORT)
        self.motion.wakeUp()


robot=NAO()
while True:
    robot.motion.move(0, 0, 0.5)

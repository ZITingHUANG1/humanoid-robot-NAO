#!/usr/bin/env python
#tutorial 2 control Giovanni Cortigiani Ziting Huang Bernhard Glas

## Server to obtain joint positions ##

import rospy
import sys
import time
import almath
from naoqi import ALProxy
from Naovid.srv import AniSpeech,AniSpeechResponse


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    while(1):
        name = 'RArm'
        frame           = 0 #FRAME_TORSO = 0
        print(motionProxy.getPosition(name, frame, True))
        time.sleep(0.05)

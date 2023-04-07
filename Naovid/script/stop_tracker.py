#!/usr/bin/env python
# Giovanni Cortigiani Ziting Huang Bernhard Glas

## Script to remove funytions initialised in our robot by other groups ##

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
    animatedSpeechProxy = ALProxy("ALAnimatedSpeech", robotIP, PORT)
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    trackerProxy = ALProxy("ALTracker", robotIP, PORT)

    trackerProxy.stopTracker()
    motionProxy.stopMove()
    print('preinstalled movements stopped')




			
		

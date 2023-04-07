#!/usr/bin/env python
#Giovanni Cortigiani Ziting Huang Bernhard Glas

## Server to perform animated speech ##

import rospy
import sys
import time
import almath
from naoqi import ALProxy
from Naovid.srv import AniSpeech,AniSpeechResponse

 #service handler
def speech(req):
    speech_string = req.speech_string
    # say the text with the local configuration
    animatedSpeechProxy.say(speech_string, configuration)
    res = AniSpeechResponse()
    res.executed = True
    return res


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    animatedSpeechProxy = ALProxy("ALAnimatedSpeech", robotIP, PORT)
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    for x in range(10) :
        motionProxy.setStiffnesses("RArm", 1.0)
        motionProxy.setStiffnesses("LArm", 1.0)

    # set the local configuration
    configuration = {"bodyLanguageMode":"contextual"}

    rospy.init_node('animated_speech_server')
    # init service
    time.sleep(0.05)

    s = rospy.Service('animated_speech', AniSpeech, speech)
    rospy.spin()

    for x in range(10) :
        motionProxy.setStiffnesses("RArm", 0.0)
        motionProxy.setStiffnesses("LArm", 0.0)





			
		

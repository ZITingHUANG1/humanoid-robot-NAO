#!/usr/bin/env python
# Giovanni Cortigiani Ziting Huang Bernhard Glas

## Server to make NAO stand up ##

import rospy
import sys
import time
import almath
from naoqi import ALProxy
from Naovid.srv import Standup,StandupResponse

 #service handler
def stand_up(req):
    if (req.fallen) :
        postureProxy.goToPosture("StandInit", 1.0)

    motionProxy.angleInterpolation("HeadPitch", -0.19, 0.3, True)
    res = StandupResponse()
    res.ready = True
    return res


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    rospy.init_node('move_joints_server')
    # init service
    time.sleep(0.05)

#    print "ready"
    s = rospy.Service('stand_up', Standup, stand_up)
    rospy.spin()
#    print("Ros is killed")

			
		

#!/usr/bin/env python

#Giovanni Cortigiani Ziting Huang Bernhard Glas

## Server to move the Head ##

import rospy
import time
import almath
import numpy
import sys
from naoqi import ALProxy
from Naovid.srv import MoveHead, MoveHeadResponse
motionProxy = 0;

#service handler

def handlemovejoints(req):

    #variables initialisation
    names = []
    names.append("HeadYaw")
    names.append("HeadPitch")
    angles = []
    angles.append(req.angle_yaw*almath.TO_RAD)
    angles.append(req.angle_pitch*almath.TO_RAD)

    angleLists = angles
    times = []
    times.append(req.time)
    times.append(req.time)
    isAbsolute = True

    motionProxy.angleInterpolation(names, angleLists, times, isAbsolute, _async=True)

    taskList = motionProxy.getTaskList()
    motionProxy.angleInterpolation(names, angleLists, times, isAbsolute, _async=True)
    if taskList!=[] :
        motionProxy.killTask(taskList[0][1])

    res = MoveHeadResponse()
    return res


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    for x in range(10) :
        motionProxy.setStiffnesses("Head", 1.0)

    rospy.init_node('move_head_server')
    #init service
    s = rospy.Service('move_head_service', MoveHead, handlemovejoints)
    rospy.spin()
			
    for x in range(10) :
        motionProxy.setStiffnesses("Head", 0.0)

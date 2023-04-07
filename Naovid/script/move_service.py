#!/usr/bin/env python
# Giovanni Cortigiani Ziting Huang Bernhard Glas

## Server to perform Joint movements ##

import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import MoveJoints2,MoveJoints2Response
from nao_control_tutorial_2.srv import MoveJoints,MoveJointsResponse

motionProxy =0;

 #service handler
def handlemovejoints(req):

    name            = req.jointname
    frame           = 0 #FRAME_TORSO = 0
    useSensorValues = True
    result          = motionProxy.getPosition(name, frame, useSensorValues)
    print "position: "
    print result

    res = MoveJointsResponse()
    res.position = result
    return res


def setposition(req):
    for i in range(10):
        motionProxy.setStiffnesses(req.jointname, 1.0)

    name            = req.jointname
    frame           = 0 #FRAME_TORSO = 0
    useSensorValues = False


    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    axisMask = 63 #position and orientation

    if req.maxspeed != 0.0 :
        print "speed2: "
        print req.maxspeed
        fractionMaxSpeed = req.maxspeed
        motionProxy.setPositions(name, frame, [req.position[0], req.position[1], req.position[2], req.orientation[0], req.orientation[1], req.orientation[2]], fractionMaxSpeed, axisMask)
    elif req.time != () :
        print "time2"
        print req.time
        times = req.time
        motionProxy.positionInterpolations(name, frame, [req.position[0], req.position[1], req.position[2], req.orientation[0], req.orientation[1], req.orientation[2]],
                                          axisMask, times)

    time.sleep(1.0)
    result = motionProxy.getPosition(name, frame, True)
    res = MoveJoints2Response(result)

    return res

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    # init service
    print "ready"
    #s = rospy.Service('move_service', MoveJoints, handlemovejoints)
    s1 = rospy.Service('move_service', MoveJoints2, setposition)
    rospy.spin()
			
		

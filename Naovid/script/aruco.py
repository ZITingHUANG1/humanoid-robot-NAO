#!/usr/bin/env python
#Giovanni Cortigiani Ziting Huang Bernhard Glas

## Server to compute homogeneous transformations between NAO camera and TORSO frame ##

import rospy
import cv2
import time
import almath
import sys
import numpy as np
from numpy.linalg import inv
from naoqi import ALProxy
from Naovid.srv import Aruco,ArucoResponse

def getpositions(req):

    frame           = 0 #FRAME_TORSO = 0
    useSensorValues = True

    # Homogeneous Coordinates of Aruco Marker in CameraBottom_optical-frame
    marker_cam = np.array([req.markerx, req.markery, req.markerz, 1])

    #Homogenous transformation matrix from CameraBottom_optical-frame to CameraBottom-frame
    Hz = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Hx = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

    Hom1 = Hz.dot(Hx)

    #Homogenous transformation matrix from CameraBottom-frame to Torso
    Mat1 = motionProxy.getTransform("CameraBottom", frame, useSensorValues)
    Hom2 = np.zeros((4,4))
    for i in range(0, 4):
           for j in range(0, 4):
               Hom2[i][j] = Mat1[4*i + j] # similar to np.reshape(Mat1,(4,4))

    #Homogenous transformation matrix from CameraBottom_optical-frame to Torso
    Hom3 = Hom2.dot(Hom1)

    # Homogeneous Coordinates of Aruco Marker in Torso-frame
    marker_torso = Hom3.dot(marker_cam)

    res = ArucoResponse()
    res.markertorsox = marker_torso[0]
    res.markertorsoy = marker_torso[1]
    res.markertorsoz = marker_torso[2]
    return res

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]

    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('aruco_server')

    print "aruco ready"
    s1 = rospy.Service('aruco', Aruco, getpositions)
    rospy.spin()
    print("Ros is killed")




			
		

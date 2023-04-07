#!/usr/bin/env python
#Giovanni Cortigiani Ziting Huang Bernhard Glas


##Server // Covid Certificate Check Service Script
##This script handles the string read in the QR code and sends the POST request to the interface.
##Furthermore, it sends all the important information as the ROS service reponse.

import rospy
import requests
import time
import almath
import sys
from Naovid.srv import CertificateCheck,CertificateCheckResponse

 #service handler
def validity_check(req):
    qrstring = req.qrstring

    r = requests.post('http://0.0.0.0:8000', json={"dcc": qrstring})
    #r.status_code
    time.sleep(1)
    data = r.json()
    val = False
    valid = False
    name = 'No valid qr-code'
    if (data.get('valid') == True):
        valid = True
        name = data['dccdata']['-260']['1']['nam']['fn'] + ' ' + data['dccdata']['-260']['1']['nam']['gn']

    print("Covid Certificate is ")
    print(valid)

    res = CertificateCheckResponse()
    res.valid = valid
    res.name = name
    return res


if __name__ == '__main__':
    rospy.init_node('move_joints_server')
    # init service
    time.sleep(0.05)

    print "ready"

    s = rospy.Service('certificate_check', CertificateCheck, validity_check)
    rospy.spin()
    print("Ros is killed")

			
		

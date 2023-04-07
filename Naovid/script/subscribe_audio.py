#!/usr/bin/env python
#tutorial 2 control Giovanni Cortigiani Ziting Huang Bernhard Glas

##Script to check functionality of ALAudioDevice


import rospy
import requests
import time
import almath
import sys
from naoqi import ALProxy
from Naovid.srv import CertificateCheck,CertificateCheckResponse


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    obj = ALProxy("ALAudioDevice", robotIP, PORT)
    from pydub import AudioSegment
    songPath = 'a.wav'
    song = AudioSegment.from_wav(songPath)
    try:
        # here is important to note that the second parameter is contigus memory audio data!
        x = obj.sendRemoteBufferToOutput(int(song.frame_count()), id(song._data))
        print "True" if x else "False"
    except Exception as e:
        print "error for buffer: "
    obj.onUnload()

			
		

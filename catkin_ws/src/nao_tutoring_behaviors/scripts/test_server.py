import os
import os.path
import sys
import random
import time
import datetime
import collections
import socket
import pdb
sys.path.append("nao_libs2")

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior

from tutorMotions import *

def robot_controller():
    NAO_PORT = 9559
    useRobot = True


    if useRobot:
        #Get the Nao's IP from file
        try:
            ipFile = open("ip.txt")
            NAO_IP = ipFile.readline().replace("\n","").replace("\r","")
        except Exception as e:
            print "Could not open file ip.txt"
            NAO_IP = raw_input("Please write Nao's IP address. ") 

        print 'nao ip:', NAO_IP
        tts = ALProxy("ALTextToSpeech", NAO_IP, 9559)
        tts.setParameter("speed", 85)


    #first connect to the NAO if -robot flag is set
    noRobot = None
    goNao = None
    if useRobot:
        try:
            print 'trying to connect nao\n'
            print NAO_IP
            print NAO_PORT
            goNao = Gesture(NAO_IP, NAO_PORT)
        except Exception as e:
            print "Could not find nao. Check that your ip is correct (%s)" %NAO_IP
            sys.exit()

        

        #Set postureProxy
        try:
            postureProxy = ALProxy("ALRobotPosture", NAO_IP, NAO_PORT)
        except Exception, e:
            print "Could not create proxy to ALRobotPosture"
            print "Error was: ", e

        motionProxy = ALProxy("ALMotion", NAO_IP, NAO_PORT)
        postureProxy.goToPosture("Sit", 0.5)
        goNao.wave()

    return goNao

if __name__ == "__main__": 
    robot_controller()
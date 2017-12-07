#!/usr/bin/env python
import rospy
import rospkg
import os
import os.path
import sys
import random
import time
import datetime
import collections
import socket
import pdb
import random
from std_msgs.msg import String
from nao_tutoring_behaviors.msg import TabletMsg
from nao_tutoring_behaviors.msg import ControlMsg


rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('nao_tutoring_behaviors')+"/scripts/nao_libs2")

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior

from tutorMotions import *
#from noRobotBehaviors import *


class RobotTutor:
    def __init__(self, goNao, noRobot):
        self.goNao = goNao
        self.noRobot = noRobot
        self.current_question_text = ""

        self.current_session = None
        self.current_question = None 

        rospy.init_node('robot_controller', anonymous = True)
        self.robot_speech_pub = rospy.Publisher('robot_speech_msg', String, queue_size=10)
        self.robot_lesson_pub = rospy.Publisher('robot_lesson_msg', String, queue_size=10)
        self.robot_inactivity_pub = rospy.Publisher('robot_inactivity_msg', String, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        rospy.Subscriber("tablet_msg", TabletMsg, self.tablet_msg_callback)
        rospy.Subscriber("tablet_lesson_msg", TabletMsg, self.tablet_lesson_msg_callback)
        rospy.Subscriber("tablet_inactivity_msg", TabletMsg, self.tablet_inactivity_msg_callback)

        rospy.Subscriber("model_decision_msg", ControlMsg, self.model_msg_callback)


    def tablet_lesson_msg_callback(self, data):                                    # this is what the robot says during an example
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.robotSpeech)      # I still have to make sure all the motions are
                                                                                   # good @TODO so do not focus too much on what is happening here
        self.robot_lesson_pub.publish(data.robotSpeech)                   
        print "Nao says: " + data.robotSpeech
        if (self.goNao!= None):
            if (data.questionNumOrPart == 0):
                speech = data.robotSpeech.split(";")
                id = self.goNao.genSpeech(speech[0])
                self.goNao.point_question() 
                #id = self.goNao.genSpeech(speech[1])
                #self.goNao.look()
               # self.goNao.speechDevice.wait(id, 0)
            elif (data.questionNumOrPart == 1):

                id = self.goNao.genSpeech(data.robotSpeech)
                self.goNao.rightExplain() 
                self.goNao.speechDevice.wait(id, 0)
            elif (data.questionNumOrPart == 5):

                id = self.goNao.genSpeech(data.robotSpeech)
                self.goNao.rightExplain() 
                self.goNao.speechDevice.wait(id, 0)
            else:
                id = self.goNao.genSpeech(data.robotSpeech)
                self.goNao.look()
                self.goNao.speechDevice.wait(id, 0)

        else:
            time.sleep(3) 

        self.robot_lesson_pub.publish("DONE")               # respond in lesson_pub rather than robot_speech_pub because the next step needs
        self.goNao.motion.rest()                            # to be triggered when I finish talking

    def tablet_inactivity_msg_callback(self, data):         # during an activity, the tablet tells the robot what to say through the inactivity message
        rospy.loginfo(rospy.get_caller_id() + "Tablet in activity: %s", data.robotSpeech)    # to not clog up the model with irrelevant messages
        self.robot_inactivity_pub.publish(data.robotSpeech) 

        print "Nao says: " + data.robotSpeech

        if (self.goNao != None):                            # right now, the robot is just saying it, but I'm going to make sure the actions are
            self.goNao.look()                               # good here @TODO
            id = self.goNao.genSpeech(data.robotSpeech) 
            self.goNao.speechDevice.wait(id, 0)
        else:
            time.sleep(1)
        
        self.robot_inactivity_pub.publish("DONE")

    def tablet_msg_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.robotSpeech)

        if (data.msgType == "START"):
            print "start wait"
            time.sleep(2)

        self.robot_speech_pub.publish(data.robotSpeech) 
        print "Nao says: " + data.robotSpeech

       

        if ("SHOWING-QUESTION" in data.msgType):       # as the robot receives the text for the question before the tablet displays it
            self.read_question()                       # it waits until it receives a message indicating that the message was shown before reading

        if (self.goNao!= None): 
            if (data.msgType == "START"):
                self.goNao.session_intro(1)            # at the start, give an intro speech
                id = self.goNao.genSpeech(data.robotSpeech)
                self.goNao.speechDevice.wait(id, 0)

            if (data.msgType == "IA"):                  # respond to an incorrect answer
                id = self.goNao.genSpeech("That's not quite right") # add in more variable responses
                self.goNao.shake()
                self.goNao.speechDevice.wait(id, 0)
            elif (data.msgType == "CA"):                # respond to a correct answer
                nods = [self.goNao.juddNelson, self.goNao.juddNelson_left, self.goNao.nod, self.goNao.lookNod, self.goNao.nodSlow, self.goNao.smallFastNod,  ]
                id = self.goNao.genSpeech("That's correct. Good job.")
                action = random.choice(nods)
                action()
                self.goNao.speechDevice.wait(id, 0)
            else:
                id = self.goNao.genSpeech(data.robotSpeech)
                self.goNao.speechDevice.wait(id, 0)
            self.goNao.motion.rest()

        self.robot_speech_pub.publish("DONE")

    def model_msg_callback(self, data):                             # respond to messages from the model
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

        if ("QUESTION" in data.nextStep and data.nextStep != "QUESTION-REPEAT"):
            self.current_question_text = data.robotSpeech;          # if we are going on to the next question, save the text of the question
            print "Setting question text"                           # to read only when it is shown

        else:                                                       # otherwise, just say what the model told us to say
            self.robot_speech_pub.publish(data.robotSpeech) 
            if (self.goNao!= None):
                id = self.goNao.genSpeech(data.robotSpeech)
                self.goNao.speechDevice.wait(id, 0)
            print "Nao says: " + data.robotSpeech
            self.robot_speech_pub.publish("DONE")


    def read_question(self):                                        # read the current question -- called when the tablet sends a message indicating
        self.robot_speech_pub.publish(self.current_question_text)   # that is has been shown
        if (self.goNao != None):
            id = self.goNao.genSpeech(self.current_question_text)
            self.goNao.look()
            self.goNao.speechDevice.wait(id, 0)
        print "Nao says: " + self.current_question_text
        self.robot_speech_pub.publish("DONE")

    def run(self):
        # init robot stuff
            while not rospy.is_shutdown():
                try: #self.behavior_test()
                    pass
                except KeyboardInterrupt:
                    if (self.goNao != None):
                        print "releaseNao"
                        self.goNao.releaseNao()
                    self.logFile.flush()
                    self.logFile.close()
                    self.conn.close()
                    self.store_session(self.current_session)
                    sys.exit(0)
            print "Shutting Down"
            if (self.goNao != None):
                print "releaseNao"
                self.goNao.releaseNao()
    
def start_robot():                      # call robot_controller to start up Nao and then run the program to wait for messages
    goNao = robot_controller()
    robot = RobotTutor(goNao, True)
    robot.run()

def robot_controller():                 # start up Nao, connect and return a reference to goNao
    NAO_PORT = 9559
    useRobot = False # change here!

    if len(sys.argv) >= 2:
        if sys.argv[1]=='-robot':
            useRobot = True


    if useRobot:
        #Get the Nao's IP from file
        try:
            ipFile = open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/ip.txt")
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

    return goNao



if __name__ == '__main__':
    try:
        start_robot()
    except rospy.ROSInterruptException:
        pass
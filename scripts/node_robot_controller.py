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
        self.sessionNum = 1

        self.current_session = None
        self.current_question = None 

        self.in_activity = False
        self.in_session = False

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
        rospy.loginfo(rospy.get_caller_id() + "I heard in lesson %s ", data.robotSpeech)      # I still have to make sure all the motions are
                                                                                   # good @TODO so do not focus too much on what is happening here
        self.robot_lesson_pub.publish(data.robotSpeech)

        if ('TUTORIAL' in data.msgType or 'TICTACTOE' in data.msgType or 'EXAMPLE' in data.msgType) :
            if ('DONE' in data.msgType):
                self.in_activity = False
                print "done with activity"

            else:
                print "setting true 1"
                self.in_activity = True

        print "Nao says: " + data.robotSpeech
        if (self.goNao!= None):
            id = self.goNao.genSpeech(data.robotSpeech)
	    self.goNao.speechDevice.wait(id, 0)
            #id = self.goNao.animated_speech_if_non_empty(data.robotSpeech)

        else:
            print "simulating waiting time"
            time.sleep(3) 

        self.robot_lesson_pub.publish("DONE")               # respond in lesson_pub rather than robot_speech_pub because the next step needs

    def tablet_inactivity_msg_callback(self, data):         # during an activity, the tablet tells the robot what to say through the inactivity message
        rospy.loginfo(rospy.get_caller_id() + "Tablet in activity: %s ", data.robotSpeech)    # to not clog up the model with irrelevant messages
        if ('PROMPT' in data.msgType):
            if self.in_activity:
                data.robotSpeech = ""

        if data.robotSpeech != "":
            self.robot_inactivity_pub.publish(data.robotSpeech) 

        if ('TUTORIAL' in data.msgType or 'TICTACTOE' in data.msgType or 'EXAMPLE' in data.msgType) :
            if ('DONE' in data.msgType):
                self.in_activity = False
                print "done with activity"
                self.robot_inactivity_pub.publish("DONE")
            else:
                print "setting true 2"
                self.in_activity = True

        print "Nao says: " + data.robotSpeech

        if (self.goNao != None):                            # right now, the robot is just saying it, but I'm going to make sure the actions are good here
            if data.robotSpeech != "":
                self.goNao.look()
                id = self.goNao.genSpeech(data.robotSpeech) 
                self.goNao.speechDevice.wait(id, 0)
        else:
            time.sleep(1)
        
        if data.robotSpeech != "":
            self.robot_inactivity_pub.publish("DONE")

    def tablet_msg_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s ", data.robotSpeech)

        if ('TUTORIAL' in data.msgType or 'TICTACTOE' in data.msgType or 'EXAMPLE' in data.msgType) :
            if ('DONE' in data.msgType):
                self.in_activity = False
                print "done with activity"
                self.robot_speech_pub.publish("DONE") #aditi

            else:
                print "setting true 3"
                self.in_activity = True

        if (data.msgType == "START"):
            print "start wait"
            time.sleep(2)

        #else:
        #    if data.robotSpeech!="": #aditi
        #        self.robot_speech_pub.publish(data.robotSpeech) 
        #        print "Nao says in tablet callback: " + data.robotSpeech

        
        if ("SHOWING-QUESTION" in data.msgType):       # as the robot receives the text for the question before the tablet displays it
            self.read_question()                       # it waits until it receives a message indicating that the message was shown before reading

        if (self.goNao!= None): 
            if (data.msgType == "START"):
                # check questionType for the session number
                self.in_session = True
                self.sessionNum = data.questionType
                id = self.goNao.session_intro(int(self.sessionNum))            # at the start, give an intro speech
                #id = self.goNao.animated_speech_return_to_neutral(data.robotSpeech)
                #if data.robotSpeech!="":
                #self.goNao.speechDevice.wait(id, 0)
                self.robot_speech_pub.publish("INTRO-DONE")
                print "Sent done after INTRO"

            elif (data.msgType == "LOAD"):
                self.in_session = True
                self.sessionNum = data.questionType
                self.robot_speech_pub.publish("INTRO-DONE")
                print "sent done after intro: not really doing intro when loading a session"

            elif (data.msgType == "IA"):   # respond to an incorrect answer
                self.goNao.incorrect_answer_speech()
                self.robot_speech_pub.publish("DONE")
                print "Sent done"
            
            elif (data.msgType == "CA"):                # respond to a correct answer
                #nods = [self.goNao.juddNelson, self.goNao.juddNelson_left, self.goNao.nod, self.goNao.lookNod, self.goNao.nodSlow, self.goNao.smallFastNod,  ]
                #action  = random.choice(nods)
                #action() #aditi - what is this?
                self.in_activity = False
                self.goNao.correct_answer_speech()
                self.robot_speech_pub.publish("DONE")
                print "Sent done"
            
            elif (data.msgType == "END"):
                self.in_session = False
                print "robot should say bye at the end of each session" #TODO: finish this block with appropriate nao behavior and speech
                self.goNao.session_goodbye(int(self.sessionNum))
            
            elif (data.msgType == "TICTACTOE-END"):
                time.sleep(3) #wait a bit here so robot can first say win/loss/tie message
                self.goNao.look()
                id = self.goNao.animated_speech_return_to_neutral(data.robotSpeech)
                if data.robotSpeech!="":
                    self.robot_speech_pub.publish("DONE")
                    print "Sent done"

            else:
                id = self.goNao.animated_speech_return_to_neutral(data.robotSpeech)
                if data.robotSpeech!="":
                    self.robot_speech_pub.publish("DONE")
                    print "Sent done"
 
        else:
            if (data.msgType == "START"):
                self.in_session = True
                self.sessionNum = data.questionType
                self.robot_speech_pub.publish("INTRO-DONE")
                print "Sent done after INTRO"

            elif (data.msgType == "LOAD"):
                self.in_session = True
                self.sessionNum = data.questionType
                self.robot_speech_pub.publish("INTRO-DONE")
                print "sent done after intro: not really doing intro when loading a session"

            elif (data.msgType == "END"):
                self.in_session = False
                print "robot should say bye at the end of each session" #TODO: finish this block with appropriate nao behavior and speech

            elif data.robotSpeech!="":
                self.robot_speech_pub.publish(data.robotSpeech) 
                print "Nao says in tablet callback: " + data.robotSpeech
                self.robot_speech_pub.publish("DONE")
                print "Sent done"

    def model_msg_callback(self, data):                             # respond to messages from the model
        rospy.loginfo(rospy.get_caller_id() + " I heard %s ", data)

        if self.in_session: #only do this stuff if the session hasn't ended

            if ("QUESTION" in data.nextStep and data.nextStep != "QUESTION-REPEAT" and data.nextStep!="QUESTION-FIRST"):
                self.current_question_text = data.robotSpeech;          # if we are going on to the next question, save the text of the question
                print "Setting question text"                           # to read only when it is shown
                if (self.goNao!= None):
                    if (not self.in_activity):
                        print "enabling next question button, but do not need to say anything"
                        self.goNao.move_on_to_next_speech()

            else:                                                       # otherwise, just say what the model told us to say
                if data.robotSpeech!="":
                    self.robot_speech_pub.publish(data.robotSpeech) 
                if (self.goNao!= None):
                    if (not self.in_activity):
                        id = self.goNao.animated_speech_return_to_neutral(data.robotSpeech)
                    #self.goNao.speechDevice.wait(id, 0)
                print "Nao says: " + data.robotSpeech
                if data.robotSpeech!="":
                    self.robot_speech_pub.publish("DONE")

        else:
            print "not doing anything because session has ended"


    def read_question(self):                                        # read the current question -- called when the tablet sends a message indicating
        if self.current_question_text!="":                          # that is has been shown
            self.robot_speech_pub.publish(self.current_question_text)   
        if (self.goNao != None):
            id = self.goNao.animated_speech_return_to_neutral(self.current_question_text)
        if self.current_question_text!="":
            print "Nao says: " + self.current_question_text
            self.robot_speech_pub.publish("DONE")
        self.in_activity = False

    def run(self):
        # init robot stuff
        while not rospy.is_shutdown():
            try: #self.behavior_test()
                pass
            except KeyboardInterrupt:
                if (self.goNao != None):
                    print "releaseNao"
                    self.goNao.releaseNao()
                
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

    #robot_flag = rospy.get_param("~robot")
    #print robot_flag
    #if robot_flag == 'robot':
    #    useRobot = True


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

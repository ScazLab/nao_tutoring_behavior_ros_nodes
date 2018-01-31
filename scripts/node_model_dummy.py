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
import json
rospack = rospkg.RosPack()
from std_msgs.msg import String
from std_msgs.msg import Int32
from nao_tutoring_behaviors.msg import TabletMsg
from nao_tutoring_behaviors.msg import ControlMsg

## This is a dummy model. It suggests tutoring behavoirs determinisitically based on the number of the question
## The relevant parts here are just those that deal with construction of model messages

class TutoringModel:
    def __init__(self):
        self.total_num_questions = 0
        self.current_question = 0
        self.level = 1

        self.pid = -1
        self.sessionNum = -1
        self.expGroup = -1
        self.difficultyGroup = -1
        self.logFile = None

        self.tries = 0

        self.attempt_times = []

        self.inSession = False

        rospy.init_node('dummy_model', anonymous = True)
        self.decisons_pub = rospy.Publisher('model_decision_msg', ControlMsg, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        rospy.Subscriber("tablet_msg", TabletMsg, self.tablet_msg_callback)     # subscribe to messages from the tablet node
        rospy.Subscriber("robot_speech_msg", String, self.robot_msg_callback)   # subscribe to messages from the robot

        # import all question jsons. Note: the tablet app should have access to the same files
        level_one_questions = []
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level1.json", 'r') as question_file_1:
            level_one_questions = json.load(question_file_1)
        level_two_questions = []
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level2.json", 'r') as question_file_2:
            level_two_questions = json.load(question_file_2)
        level_three_questions = []
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level3.json", 'r') as question_file_3:
            level_three_questions = json.load(question_file_3)
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level4.json", 'r') as question_file_4:
            level_four_questions = json.load(question_file_4)
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level5.json", 'r') as question_file_5:
            level_five_questions = json.load(question_file_5)
        
        self.questions = [[], level_one_questions, level_two_questions, level_three_questions]
        self.harder_questions = [[], [], [], level_three_questions, level_four_questions, level_five_questions]

    def repeat_question(self):                                  # send this message to have the student try the same question again
        control_message = ControlMsg()                          # with no tutoring behavior
        control_message.nextStep = "QUESTION-REPEAT"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = "Try that again."

        self.decisons_pub.publish(control_message)
        print "sent:" , control_message
        

    def next_question(self):                                    # indicates that the student should move on to the next question
        self.total_num_questions += 1                           # keeps track of the total number of questions student has seen 
        self.tries = 0                                          # reset the number of attempts on the question                                      
        self.level = self.total_num_questions % 3
        if self.difficultyGroup==0:
            self.level += 1
        else:
            self.level += 3
        #self.current_question += 1                                 # whether after a correct response or after enough incorrect attempts
                                                                # without a message, the tablet will not display a new question
        if self.total_num_questions % 3==0:
            self.current_question += 1


        if (self.current_question >= len(self.questions[self.level])):
            print "this should only happen if student has really finished all questions"        
            #return self.question_next_level()

        control_message = ControlMsg()
        control_message.nextStep = "QUESTION-NEXT"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']  # we give the text for this question 
                                                                                                            # to the robot here
        print self.level, self.current_question, self.questions[self.level][self.current_question]
        
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def question_next_level(self):                              # indicates that the student should move to the next level
        self.tries = 0                                          # so we increase the level number and go to question 1
        self.current_question = 1
        self.level += 1

        control_message = ControlMsg()
        control_message.nextStep = "QUESTION-LEVEL"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']  # give the text for the question to the robot
    
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def send_next_question(self): # THIS IS NOT USED
        control_message = ControlMsg()
        control_message.nextStep = "QUESTION-NEXT"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']

    def tic_tac_toe_break(self):                                # trigger a game of tic tac toe for a break
        control_message = ControlMsg()                          # the robot message here is the speech for the beginning of the game
        control_message.nextStep = "TICTACTOE"
        control_message.robotSpeech = "Lets play a game of tic-tac-toe. You will be exes, and I will be ohs. You can go first. Click any square on the board."
        
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message


    def give_hint(self):                                        
        control_message = ControlMsg()                          
        control_message.nextStep = "SHOWHINT"                   
        control_message.questionNum = self.current_question     
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""
        
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def give_structure_hint(self):                              # give a hint in the form of the structure of the problem
        control_message = ControlMsg()
        control_message.nextStep = "SHOWSTRUCTURE"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = "Let's look at how to structure a solution"
        
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def give_think_aloud(self):                                 # ask the student to think aloud
        control_message = ControlMsg()
        control_message.nextStep = "THINKALOUD"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = "What is the first thing you want to do to solve this problem?"   # give the robot text here because node_tablet 
                                                                                                        # doesn't do anything with this
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message


    def give_example(self):                                     # give a worked example - what example is given is determined by node_tablet code
        control_message = ControlMsg()                          # and is based on the level. node_tablet controls all tablet and robot actions
        control_message.nextStep = "SHOWEXAMPLE"                # during this
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def give_tutorial(self):                                    # give an interactive tutorial. like the worked example, this is controled by node_tablet
        control_message = ControlMsg()
        control_message.nextStep = "SHOWTUTORIAL"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message     

    def form_observation(self, msgType, speed):
        obs = ""
        if msgType == 'CA':
            obs += "R-"
        elif msgType == 'IA':
            obs += "W-"
        else:
            print "should not be here"
            return ""

        speed = float(speed) / 1000.0
        print "speed in seconds is: " + str(speed)

        if speed > 60:
            obs += "med"
        else:
            obs += "slow"

        return obs


    def tablet_msg_callback(self, data):                                        # respond to tablet messages by triggering the next behavior
        rospy.loginfo(rospy.get_caller_id() + " From Tablet, I heard %s ", data)    # the code here is just based off the question number, but
                                                                                # the real model can similarly respond to whether or not the 
        if (data.msgType == 'CA'): # respond to correct answer                  # answer was correct and call one of these functions to produce a behavior
            self.next_question()                                                
        elif (data.msgType == 'IA'): # respond to incorrect answer
            self.tries +=1
            if(self.tries >= 3):
                self.next_question()
            elif (data.questionNumOrPart % 4 == 0):
                self.give_example()
                #self.tic_tac_toe_break() #ADITI: commenting out for now
            elif (data.questionNumOrPart % 4 == 1):
                self.give_tutorial()
            elif (data.questionNumOrPart % 4 == 2):
                self.give_hint()
            elif (data.questionNumOrPart % 4 == 3):
                self.give_example()
            else:
                self.give_think_aloud()

        elif (data.msgType == "TICTACTOE-WIN" or data.msgType == "TICTACTOE-LOSS"):  # here I respond to the end of a game by going to the same
            self.repeat_question()                                                   # question, but you could return to the next one
        
        elif("SHOWEXAMPLE" in data.msgType):                                         
            pass
        
        elif(data.msgType == 'START'):
            print "MODEL RECEIVED START MESSAGE FROM TABLET_MSG --------------> setting up session"
            self.inSession = True
            self.pid = int(data.questionNumOrPart)
            self.sessionNum = int(data.questionType)
            self.expGroup = int(data.robotSpeech)
            self.difficultyGroup = int(data.otherInfo)
            

            fileString = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/"+"P"+str(self.pid)+"_S"+str(self.sessionNum)+".txt"
            print fileString
            if os.path.exists(fileString):
                self.logFile = open(fileString, "a")
            else:
                self.logFile = open(fileString, "w+")
            self.logFile.write("PARTICIPANT_ID,EXP_GROUP,SESSION_NUM,TIMESTAMP,QUESTION_NUM,TYPE,OTHER_INFO\n");

            if self.sessionNum == 1: 
                self.attempt_times = []
                if self.difficultyGroup == 1:
                    print "harder difficulty group"
                    self.questions = self.harder_questions
                    self.current_level = 3
                else:
                    print "easier difficulty group"
            else:
                self.attempt_times = [] #TODO: change this to initialize it from the param save file
                saveFileString = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/" + "P"+str(self.pid)+"_save.json"
                if os.path.exists(saveFileString):
                    with open(saveFileString) as param_file:
                        params = json.load(param_file)

                    self.difficultyGroup = params["difficultyGroup"]
                    num_problems = params["numProblemsCompleted"]
                    self.total_num_questions = num_problems #this tracks total number of q's over all sessions
                    #self.current_question = params["currentQuestionIndex"]
                    if self.difficultyGroup == 1:
                        self.questions = self.harder_questions
                        self.current_level = (num_problems % 3) + 3
                    else:
                        self.current_level = (num_problems % 3) + 1
                    self.current_question = num_problems / 3
                else:
                    print "error: tried to open param save file when it didnt exist"
            #self.send_first_question()
            #self.next_question() #aditi - trying this instead since send_first_question does not exist
        elif(data.msgType == 'END'):
            self.inSession = False
            print "End of session - should try to save whatever info is needed to restart for next session"
            self.save_params()
            


    def save_params(self):
        saveFileString = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/" + "P"+str(self.pid)+"_save.json"
        self.save_file = open(saveFileString, "w+")
        save_params = {"difficultyGroup": self.difficultyGroup,
                       "numProblemsCompleted": self.total_num_questions}
        param_string = json.dumps(save_params, indent=4)
        self.save_file.write(param_string)
        self.save_file.flush()
        self.save_file.close()

    

    def robot_msg_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " From Robot, I heard %s ", data)      # this model does nothing with robot messages, but it could
                                                                                     # do so in this function

    def run(self):

        while not rospy.is_shutdown():
            try:
                pass

            except KeyboardInterrupt:
                self.logFile.flush()
                self.logFile.close()
                if self.inSession:
                    self.save_params()
                #self.conn.close()
                #self.store_session(self.current_session)
                sys.exit(0)



def run_model():
    model = TutoringModel()
    model.run()

if __name__ == '__main__':
    try:
        run_model()
    except rospy.ROSInterruptException:
        pass

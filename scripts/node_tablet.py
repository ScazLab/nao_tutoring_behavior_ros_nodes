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
import example_generation
rospack = rospkg.RosPack()
from std_msgs.msg import String
from std_msgs.msg import Int32
from nao_tutoring_behaviors.msg import TabletMsg
from nao_tutoring_behaviors.msg import ControlMsg

level_one_hints = ["Remember that division is the opposite of multiplication.", "What number times the smaller number equals the bigger number?"]

doing_question_state = 1
break_state = 2
lesson_state = 3
after_action_state = 4

class TabletSession:
    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.pid = -1
        self.sessionNum = -1
        self.expGroup = -1
        self.difficultyGroup = -1 #0 or 1 for easier or harder
        self.logFile = None
        self.state = 0
        self.conn = None

        # holds session data for 'this' user
        self.current_question = 0
        self.current_level = 1
        self.current_session = None  # will be initialized when START message parsed
        self.example_step = 0
        self.tutorial_step_attempts = 0
        self.example_number = 0
        self.tutorial_number = 0
        self.showing_hint = False

        self.lessons = []

        # open files with tutorials and questions. 
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level1examples.json", 'r') as example_file1:
            self.lessons.append(json.load(example_file1))
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level2examples.json", 'r') as example_file2:
            self.lessons.append(json.load(example_file2))
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level3examples.json", 'r') as example_file3:
            self.lessons.append(json.load(example_file3))
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level4examples.json", 'r') as example_file4:
            self.lessons.append(json.load(example_file4))
        with open(rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/level5examples.json", 'r') as example_file5:
            self.lessons.append(json.load(example_file5))

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

        self.questions = [[], level_one_questions, level_two_questions, level_three_questions, level_four_questions, level_five_questions]
        #self.harder_questions = [[], [], [], level_three_questions, level_four_questions, level_five_questions]

        # we cycle through the available examples and tutorials in each level
        self.current_example = self.lessons[self.current_level - 1]["Examples"][self.example_number]
        self.current_tutorial = self.lessons[self.current_level - 1]["Tutorials"][self.tutorial_number]
        self.current_question_steps = [];

        rospy.init_node('tablet_node', anonymous=True)
        #self.tablet_pub = rospy.Publisher('tablet_msg', TabletMsg, queue_size=10)
        self.tablet_pub = rospy.Publisher('tablet_msg', TabletMsg, queue_size=10)
        self.tablet_lesson_pub = rospy.Publisher('tablet_lesson_msg', TabletMsg, queue_size=10)
        self.tablet_inactivity_pub = rospy.Publisher('tablet_inactivity_msg', TabletMsg, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        # subscribe to messages
        rospy.Subscriber("model_decision_msg", ControlMsg, self.model_msg_callback) # this receives messages from the model
        rospy.Subscriber("robot_speech_msg", String, self.robot_msg_callback)       # this receives messages from the robot indicating speech
        rospy.Subscriber("robot_lesson_msg", String, self.robot_lesson_callback)    # this message is received from the robot DURING worked examples showing progressive 
                                                                                    # steps of box tutorial. It triggers the next step of the example to be shown.
        rospy.Subscriber("robot_inactivity_msg", String, self.robot_msg_callback)


    # for normal messages indicating robot speech, send a message to the tablet indicating when the robot starts and stops speaking. Buttons on the tablet 
    # will be disabled during this time
    def robot_msg_callback(self, data):
        rospy.loginfo("FROM ROBOT_MSG_CALLBACK IN NODE_TABLET: I heard %s ", data.data)
        if (data.data == "DONE"):
            print "FROM ROBOT_MSG_CALLBACK IN NODE_TABLET, SENDING SPEAKING-END TO TABLET\n"
            returnMessage = "SPEAKING-END"
            #print "robot was done"
        else:
            print "FROM ROBOT_MSG_CALLBACK IN NODE_TABLET, SENDING SPEAKING-START TO TABLET\n"
            returnMessage = "SPEAKING-START"

        time.sleep(2) #wait a bit before sending message
        self.conn.send(returnMessage+"\n") #send robot message back to tablet

    # handle messages from the model and send the appropriate messages to the tablet and robot to complete the indicated tutoring behavior
    def model_msg_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' From Model: I heard %s ', data)

        if (self.state == lesson_state or self.state == break_state):         # this shouldn't happen, but if the model tells us to do something while we are already 
            print "ignoring message because we are doing something else now"    # doing another activity, ignore the message
        else:
            self.current_question = data.questionNum #aditi - we need to get this info from the model
            self.current_level = data.questionLevel #aditi - we need to get this info from the model

            if (data.nextStep == "SHOWEXAMPLE"):                                  # show an example based on the level of the question
                if (self.current_level == 1):                                     # if in level one, show an easy example showing multiplication and division facts 
                    self.run_easy_example()
                else:
                    self.example_step = 0                                           # in other levels, show an example of filling in the box structure
                    self.run_example()

            elif ("QUESTION" in data.nextStep):                                 # send the next question number to the tablet -- it also has access to the JSON
                if (self.current_level != data.questionLevel):                    # so only the level and question number need to be sent
                    self.tutorial_number = 0
                    self.example_number = 0
                self.showing_hint = False
                self.current_question = data.questionNum
                self.current_level = data.questionLevel
                self.example_step = -1
                if ("REPEAT" not in data.nextStep):
                    if "FIRST" in data.nextStep:
                        messageToTablet = "QUESTIONFIRST;" + str(self.current_level) + ";" + str(self.current_question)
                    else:
                        messageToTablet = "QUESTION;" + str(self.current_level) + ";" + str(self.current_question)
                    self.conn.send(messageToTablet+"\n") #send model message back to tablet
                    print "Sent message to tablet: " + messageToTablet

            elif ("SHOWSTRUCTURE" in data.nextStep):                            # sending SHOWSTRUCTURE to the tablet will cause it to show the 
                messageToTablet = "SHOWSTRUCTURE;"                                # boxes corresponding to the current question
                self.conn.send(messageToTablet+"\n") 
                print "Sent message to tablet: " + messageToTablet
                (robot_speech, tablet_steps, all_answers) = example_generation.get_box_steps(self.current_tutorial["numerator"], self.current_tutorial["denominator"])
                self.current_question_steps = tablet_steps
                self.showing_hint = True

            elif ("THINKALOUD" in data.nextStep):                               # the tablet does nothing during thinkaloud so do not send message to tablet
                return 

            elif ("SHOWHINT" in data.nextStep):                                
                self.show_hint()

            elif ("SHOWTUTORIAL" in data.nextStep):                             # show an interactive tutorial depending on the level
                self.example_step = 0
                self.tutorial_step_attempts = 0
                if (self.current_level == 1):                                     # in level one, show balls in boxes to represent the division fact
                    self.run_easy_tutorial()
                else:                                                             # in other levels, show a box structure and provide feedback
                    self.run_boxes_tutorial("")

            elif ("TICTACTOE" in data.nextStep):
                messageToTablet = data.nextStep 
                self.conn.send(messageToTablet+"\n") 
                print "Sent message to tablet: " + messageToTablet

            else:                                                               
                messageToTablet = data.nextStep 
                self.conn.send(messageToTablet+"\n") #send model message back to tablet
                print "Sent message to tablet: " + messageToTablet

    def show_hint(self):
        # hint for harder levels is the structure. If the structure is already showing (due to a previous hint for example), then a step 
        # is filled in 
        tablet_msg = TabletMsg()
        tablet_msg.msgType = 'SHOWHINT'
        tablet_msg.questionNumOrPart = self.current_question
        tablet_msg.otherInfo = ""

        if (self.current_level > 1):
            if (self.showing_hint == False):
                print "current level and question are: " + str(self.current_level) + ", " + str(self.current_question)
                num = int(self.questions[self.current_level][self.current_question]["Numerator"])
                den = int(self.questions[self.current_level][self.current_question]["Denominator"])
                print num, den
                (robot_speech, tablet_steps, all_answers) = example_generation.get_box_steps(num, den)
                self.current_question_steps = tablet_steps

                self.showing_hint = True
                messageToTablet = "SHOWSTRUCTURE;"                                
                self.conn.send(messageToTablet+"\n") 
                print "Sent message to tablet: " + messageToTablet
                self.example_step = 1
                tablet_msg.robotSpeech = "This is how you could structure the solution to this problem."
            else: 
                msg_to_tablet = "FILLSTRUCTURE;" + self.current_question_steps[self.example_step]  # tablet a message to fill in the boxes for them
                self.example_step += 1
                tablet_msg.robotSpeech = "Let me fill in a couple boxes for you. You can continue the problem from there."
                self.conn.send(msg_to_tablet + "\n")                                                
                print "sent: ", msg_to_tablet
      
        else: # TODO: fix level one hints so that they are not just the same phrases over and over --> move to examples file
            if (self.showing_hint):
                hint = level_one_hints[1]
            else:
                hint = level_one_hints[0]
                self.showing_hint = True
            tablet_msg.robotSpeech = hint
            msg_to_tablet = "SHOWTEXTHINT;" + hint + ";"                                        # hint for level one is just text right now, but could also
            self.conn.send(msg_to_tablet + "\n")                                                # be showing the easy tutorial picture by substituting run_easy_tutorial() here
            print "sent: ", msg_to_tablet
      
        self.tablet_inactivity_pub.publish(tablet_msg)



    # in an easy example, display the multiplication and division facts that represent the problem
    # it just shows up as text, and the robot says it
    def run_easy_example(self): 
        self.tutorial_number = random.randint(0, len(self.lessons[0]["Tutorials"])-1)                                                               
        numerator = self.lessons[0]["Tutorials"][self.tutorial_number]["numerator"]
        denominator =  self.lessons[0]["Tutorials"][self.tutorial_number]["denominator"]      
        quotient = numerator / denominator                                                    

        text = str(denominator) + " x " + str(quotient) + " = " + str(numerator)
        msg_to_tablet = "SHOWTEXTHINT;" + text + ";"
        self.conn.send(msg_to_tablet + "\n")
        print "sent: ", msg_to_tablet

        text = str(numerator) + " / "  + str(denominator) + " = " + str(quotient)
        msg_to_tablet = "SHOWTEXTHINT;" + text + ";"
        self.conn.send(msg_to_tablet + "\n")
        print "sent: ", msg_to_tablet


        text = "Try the same thing with this problem."
        msg_to_tablet = "SHOWTEXTHINT;" + text + ";"
        self.conn.send(msg_to_tablet + "\n")
        print "sent: ", msg_to_tablet


        robotSpeech = "Let's look at this similar problem together. " + str(denominator) + " times " + str(quotient) + " equals " + str(numerator) + " so " + str(numerator) + " divided by "  + str(denominator) + " equals " + str(quotient) + ". Now try the question you were working on before."

        tablet_msg = TabletMsg()
        tablet_msg.msgType = 'SHOWEXAMPLE'
        tablet_msg.questionNumOrPart = self.example_step
        tablet_msg.otherInfo = "otherInfo"
        tablet_msg.robotSpeech = robotSpeech
        self.tablet_pub.publish(tablet_msg)

        self.example_step = -1
        #self.tutorial_number = (self.tutorial_number + 1) % len(self.lessons[self.current_level - 1]["Tutorials"])


    # this handles messages within worked examples - when the robot stops talking, 
    # this indicates that we can move onto the next step of the example
    def robot_lesson_callback(self, data):
        print "with " + str(self.example_step) + " in robot lesson callback, heard: " + data.data           
        #print data
                                                                                                
        if (data.data == 'DONE'):
            # send next part of example -- do not enable buttons on tablet
            self.run_example()
        
        elif self.example_step==1: #aditi - changed from else to elif
            print "FROM ROBOT_LESSON_CALLBACK IN NODE_TABLET, SENDING SPEAKING-START TO TABLET\n"
            returnMessage = "SPEAKING-START" 
            self.conn.send(returnMessage+"\n") #send robot message back to tablet
        else:
            print "FROM ROBOT_LESSON_CALLBACK IN NODE_TABLET: SHOULD BE IN THE MIDDLE OF THE WORKED EXAMPLE, NOT SENDING STPEAKING-START\n"

    def robot_inactivity_callback (self, data):
        print "robot says " + data.data
        if (data.data == 'DONE'):
            # enable board buttons by sending message
            print "FROM ROBOT_INACTIVITY_CALLBACK IN NODE_TABLET, SENDING SPEAKING-END TO TABLET\n"
            self.conn.send("SPEAKING-END\n") #send robot message back to tablet
        
        else:
            print "FROM ROBOT_INACTIVITY_CALLBACK IN NODE_TABLET, SENDING SPEAKING-START TO TABLET\n"
            self.conn.send("SPEAKING-START\n") #send robot message back to tablet

    
    def run_easy_tutorial(self, status = ""):                           # to run an easy tutorial, we show balls in boxes and confirm if the
        msg_type = "SHOWTUTORIAL;"                                        # answers to the middle steps are correct. This function is called at
        tablet_msg = TabletMsg()
        tablet_msg.msgType = msg_type                                     # the beginning due to a model message but also in intermediate steps on 
        tablet_msg.questionNumOrPart = self.example_step                  # receipt of messages with answer attempts from the tablet
        tablet_msg.otherInfo = "otherInfo"

        if (self.example_step == -1):
            return

        if (self.example_step == 0):                                       # display the images in the first step
            self.example_step = 1
            self.tutorial_number = random.randint(0, len(self.lessons[0]["Tutorials"])-1)
            messageToTablet = "SHOWEASYTUTORIAL;" + str(self.lessons[0]["Tutorials"][self.tutorial_number]["numerator"]) + '-' + str(self.lessons[0]["Tutorials"][self.tutorial_number]["denominator"])
            self.conn.send(messageToTablet+"\n") #send model message back to tablet
            print "Sent message to tablet: " + messageToTablet
            tablet_msg.robotSpeech = "Look at this example. There are " + str(self.lessons[0]["Tutorials"][self.tutorial_number]["numerator"]) + " balls in " + str(self.lessons[0]["Tutorials"][self.tutorial_number]["denominator"]) + " boxes. How many are in each box? Fill in the steps."

        if (status.startswith("INCOMPLETE") or status.startswith("INCORRECT")):     # if the student has gotten the answer wrong enough times,  
            if (self.tutorial_step_attempts > 2):                                     # we send them the answer to a step
                # move on to next step
                self.tutorial_step_attempts = 0
                tablet_msg.robotSpeech = "Here is the answer to this step"
                messageToTablet = "FILLSTRUCTURE;EASY;" + str(self.example_step) + "-" + str(self.lessons[0]["Tutorials"][self.tutorial_number]["numerator"]/self.lessons[0]["Tutorials"][self.tutorial_number]["denominator"])
                print "Sent message to tablet: " + messageToTablet
                self.conn.send(messageToTablet+"\n") #send model message back to tablet
                self.example_step += 1

            else: 
                if ("INCOMPLETE" in status):
                    tablet_msg.robotSpeech = "Fill in all the boxes."
                else :
                    tablet_msg.robotSpeech = "Not quite. Try filling in the boxes again. How many balls are there in each box?"
                self.tutorial_step_attempts += 1                                        # otherwise tell them to try again
        
        elif (status.startswith("DONE") or status.startswith("CORRECT")):            # at the end of the tutorial tell them to go back to the original problem
            self.example_step = -1
            tablet_msg.msgType = 'TUTORIAL-DONE'
            self.tutorial_step_attempts = 0
            #self.tutorial_number = (self.tutorial_number + 1) % len(self.lessons[self.current_level - 1]["Tutorials"])
            tablet_msg.robotSpeech = "Great! You finished this problem. Now go back to the one you were working on."
        
        self.tablet_inactivity_pub.publish(tablet_msg)


  
    def run_boxes_tutorial(self, status):                                         # a box tutorial shows the box structure for a problem and asks the 
        msg_type = "SHOWTUTORIAL;"                                                # student to fill in intermediate steps
        tablet_msg = TabletMsg()                                                  # this function is called at the begining due to the model message
        tablet_msg.msgType = msg_type                                             # and on receipt of answer attempts from the tablet
        tablet_msg.questionNumOrPart = self.example_step
        tablet_msg.otherInfo = "otherInfo"

        if (self.example_step == -1):
            return

        #self.current_tutorial = self.lessons[self.current_level - 1]["Tutorials"][self.tutorial_number]

        print "running boxes tutorial", self.example_step

        if (self.example_step == 0):
            self.example_step += 1

            # get the steps for this example from the example_generation code
            self.tutorial_number = random.randint(0, len(self.lessons[self.current_level-1]["Tutorials"])-1)
            self.current_tutorial = self.lessons[self.current_level - 1]["Tutorials"][self.tutorial_number]
            (robot_speech, tablet_steps, all_answers) = example_generation.get_box_steps(self.current_tutorial["numerator"], self.current_tutorial["denominator"])
            self.current_tutorial["SpokenText"] = robot_speech
            self.current_tutorial["TabletSteps"] = tablet_steps
            self.current_tutorial["Answers"] = all_answers

            # send a message to the tablet to show the structure (for a tutorial) with the numerator and denominator and all the answers it should expect
            # this is how it will verify the student's work
            msg_to_tablet = "SHOWSTRUCTURE-TUTORIAL;" + str(self.current_tutorial["numerator"]) + "-" + str(self.current_tutorial["denominator"]) + ";" + self.current_tutorial["Answers"]
            tablet_msg.robotSpeech = "Let's look at this example with the box structure. What is " + str(self.current_tutorial["numerator"]) + " divided by " + str(self.current_tutorial["denominator"]) + " Try to fill in the first step of boxes."
            self.conn.send(msg_to_tablet + "\n")
            print "sent" + msg_to_tablet
            self.tablet_inactivity_pub.publish(tablet_msg)

        elif (self.example_step < len(self.current_tutorial)):                                          # after the first step respond to student attempts
            if (status.startswith("INCOMPLETE") or status.startswith("INCORRECT")):
                if (self.tutorial_step_attempts > 2):                                                         # if the student has been wrong enough times, send the
                    msg_to_tablet = "FILLSTRUCTURE;"                                                            # tablet a message to fill in the boxes for them
                    self.conn.send(msg_to_tablet + "\n")                                                        # Note: "FILLSTRUCTURE;" with no specified numbers will fill in
                    print "sent" + msg_to_tablet                                                                # all enabled boxes (i.e. the current step), but numbers can be
                    tablet_msg.robotSpeech = "Here is the answer to this step."                                 # be specified to indicate the boxes to fill in (see Readme)
                    
                    self.tutorial_step_attempts = 0
                    self.tablet_inactivity_pub.publish(tablet_msg)
                
                else:
                    if (status.startswith("INCOMPLETE")) :                                                      # otherwise, encourage them to try again
                        tablet_msg.robotSpeech = "Fill in all the boxes for this step."
                    else:
                        tablet_msg.robotSpeech = "Not quite. Try filling in the boxes again."
                    self.tutorial_step_attempts += 1
                    self.tablet_inactivity_pub.publish(tablet_msg)

            if (status.startswith("CORRECT")):                                                              # or move on to the next step if correct
                self.tutorial_step_attempts = 0
                self.example_step += 1
                print self.example_step
                tablet_msg.robotSpeech = "Very good."
                self.tablet_inactivity_pub.publish(tablet_msg)

        else:
            print "done with tutorial"                              # at the end of the tutorial, tell them to go back to the original problem
            self.example_step = -1                                  # and reset the steps / increase the current tutorial
            tablet_msg.robotSpeech = "Great! You finished this problem. Now go back to the one you were working on."
            #self.tutorial_number = (self.tutorial_number + 1) % len(self.lessons[self.current_level - 1]["Tutorials"])
            tablet_msg.msgType = 'TUTORIAL-DONE'
            self.tablet_inactivity_pub.publish(tablet_msg)



    def run_example(self):                                        # an example shows the box structure for a problem and then fills in
                                                                  # the steps in succession. This function is called from the model message
        msg_type = "SHOWEXAMPLE;"                                 # and then from robot_lesson_callback() (aka when the robot has finished 
                                                                  # saying the previous step)
        tablet_msg = TabletMsg()
        tablet_msg.msgType = 'SHOWEXAMPLE'
        tablet_msg.questionNumOrPart = self.example_step
        tablet_msg.otherInfo = "otherInfo"

        #print "running example", self.example_step, len (self.current_example["SpokenText"])
        #self.current_example = self.lessons[self.current_level - 1]["Examples"][self.example_number]

        
        if (self.example_step < 0):
            return

        if (self.example_step > 0 and self.example_step < len (self.current_example["SpokenText"])):
            tablet_msg.robotSpeech = self.current_example["SpokenText"][self.example_step]
  
        if (self.example_step == 0):                               # in the first step, get the steps for this example from the generation code 
            self.example_number = random.randint(0, len(self.lessons[self.current_level-1]["Examples"])-1)
            self.current_example = self.lessons[self.current_level - 1]["Examples"][self.example_number]
            (robot_speech, tablet_steps, all_answers) = example_generation.get_box_steps(self.current_example["numerator"], self.current_example["denominator"])
            self.current_example["SpokenText"] = robot_speech
            self.current_example["TabletSteps"] = tablet_steps
            #print "IN WORKED EXAMPLE, TABLET STEPS ARE: " + str(self.current_example["TabletSteps"])
            self.current_example["Answers"] = all_answers

            self.example_step += 1                                   # and then tell the tablet to display the structure for this problem
            msg_to_tablet = "SHOWSTRUCTURE;" + str(self.current_example["numerator"]) + "-" + str(self.current_example["denominator"]) 
            self.conn.send(msg_to_tablet + '\n')
            print "sent: " + msg_to_tablet
            self.tablet_lesson_pub.publish(tablet_msg)

        elif (self.example_step < len(self.current_example["TabletSteps"])):                          # in future steps, send the next step of the problem
            msg_to_tablet = "FILLSTRUCTURE;" + self.current_example["TabletSteps"][self.example_step]   # to the tablet and to the robot
            self.conn.send(msg_to_tablet + "\n")                                                        # to fill in the boxes and have the robot speak
            print "sent: " + msg_to_tablet                                                                # through the steps
            self.example_step += 1
            self.tablet_lesson_pub.publish(tablet_msg)

        elif (self.example_step < len(self.current_example["SpokenText"])):
            print "sending message only to robot: " + tablet_msg.robotSpeech
            self.example_step += 1
            self.tablet_lesson_pub.publish(tablet_msg)
        else:
            self.example_step = -1                                   # send a message to robot and model when the example is done
            tablet_msg.msgType = 'EXAMPLE-DONE'
            self.tablet_pub.publish(tablet_msg)
            #self.example_number = (self.example_number + 1) % len(self.lessons[self.current_level - 1]["Examples"])


        
    def run(self):
        # connect to tablet

        BUFFER_SIZE = 1024  

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        print s.getsockname()
        s.listen(1)
        print 'Waiting for client connection...'

        try:
            self.conn, addr = s.accept()
            self.conn.settimeout(None)
            print 'Connection address:', addr
            msg = self.conn.recv(BUFFER_SIZE)
            print msg
        except KeyboardInterrupt:
            sys.exit()

        sessionEnded = False
            
        while not rospy.is_shutdown():                  # this code handles messages from the tablet
            #try:
            # get message from tablet and parse
            try:
                msg = self.conn.recv(BUFFER_SIZE)
                if not msg: break
                print "======================================================================="
                print "received msg: ", msg

                msgType = msg.split(";")[0]
                print msgType

                tablet_msg = TabletMsg()
                tablet_msg.msgType = msgType
                tablet_msg.questionNumOrPart= self.current_question
                tablet_msg.robotSpeech = ""
                tablet_msg.otherInfo = ""

                if msgType == 'START':                  # this message is received at the beginning. 
                    self.pid = int(msg.split(";")[1])
                    self.sessionNum = int(msg.split(";")[2])
                    self.expGroup = int(msg.split(";")[3])
                    self.difficultyGroup = int(msg.split(";")[4])
                    print "starting partipant: " + str(self.pid) + " on session number: " + str(self.sessionNum)
                    tablet_msg.questionNumOrPart = self.pid 
                    tablet_msg.questionType = str(self.sessionNum)
                    tablet_msg.robotSpeech = str(self.expGroup)
                    tablet_msg.otherInfo = str(self.difficultyGroup) 
                    #if str(difficultyGroup) == '1':
                        #self.questions = self.harder_questions
                        #self.current_level = 3
                    self.state = doing_question_state     
                    self.tablet_pub.publish(tablet_msg)

                elif ('SHOWING-QUESTION' in msgType ):  # when the next question is shown, the tablet sends this message - this helps the robot 
                    #if len(msg.split(";")) > 1:
                    #    tablet_msg.robotSpeech = msg.split(";")[1]
                    self.tablet_pub.publish(tablet_msg)   # know when to speak, and tells the model in case the model wants to time the answer

                elif msgType == 'CA' or msgType == 'IA' and self.state != lesson_state and self.state != break_state:
                    # the tablet is reporting an answer
                    # pass this information on by publishing a message -- the model will not give a tutoring behavior if it does not have this
                    attempt = msg.split(";")[1]
                    timing = msg.split(";")[2]
                    print "ATTEMPT IS: " + attempt
                    print "TIMING FOR ATTEMPT IS: " + timing
                    tablet_msg.robotSpeech = ""
                    tablet_msg.otherInfo = attempt+"-"+timing
                    self.state = after_action_state
                    self.tablet_pub.publish(tablet_msg)

                elif msgType.startswith('TICTACTOE'):     # continue playing tic tac toe game
                    tablet_msg.robotSpeech = msg.split(";")[3]
                    if (msgType == "TICTACTOE-WIN" or msgType == "TICTACTOE-LOSS"):
                        self.tablet_pub.publish(tablet_msg)
                        self.state = after_action_state
                        msg_to_tablet = "TICTACTOE-END;" + str(self.current_level) + ";" + str(self.current_question)
                        self.conn.send(msg_to_tablet+ "\n")                                                        # to fill in the boxes and have the robot speak
                        print "sent" + msg_to_tablet 
                        continue

                    else:
                        self.state = break_state
                        tablet_msg.robotSpeech = msg.split(";",4)[3]
                        self.tablet_inactivity_pub.publish(tablet_msg)
                      
                        returnMessage = msgType 
                    
                    self.conn.send(returnMessage + "\n")
                    print "sent: " + returnMessage
                
                elif msgType.startswith('TUTORIAL-STEP'):         # this message is received when the student has made an attempt in a tutorial
                    if 'EASY' in msgType:                           # pass the message on to the appropriate tutorial function to either move on
                        self.run_easy_tutorial(msg.split(";")[1])     # to the next step or fill in some answers
                    else:
                        print "tutorial : " + msg.split(";")[1]
                        self.run_boxes_tutorial(msg.split(";")[1])

                elif msgType == 'END':
                    self.tablet_pub.publish(tablet_msg)

            except KeyboardInterrupt:
                    #self.logFile.flush()
                    #self.logFile.close()
                    self.conn.close()
                    #self.store_session(self.current_session)
                    sys.exit(0)

            #except KeyboardInterrupt:
                    #self.logFile.flush()
                    #self.logFile.close()
                    #self.conn.close()
                    #self.store_session(self.current_session)
                    #sys.exit(0)



def tablet_message_connection():
    #if len(sys.argv) >=3:
        #TCP_IP = sys.argv[1]
        #TCP_PORT = int(sys.argv[2]) 
    # TCP_IP = rospy.get_param("host")
    # TCP_PORT = rospy.get_param("port")

    name = socket.gethostname()
    TCP_IP = socket.gethostbyname(str(name))
    print str(TCP_IP)
    if str(TCP_IP).startswith('127'):
        #need this for getting IP on linux machine
        TCP_IP = socket.gethostbyname(str(name)+".local")
    TCP_PORT = 9090

    session = TabletSession(TCP_IP, TCP_PORT)
    session.run()    


if __name__ == '__main__':
    try:
        tablet_message_connection()
    except rospy.ROSInterruptException:
        pass

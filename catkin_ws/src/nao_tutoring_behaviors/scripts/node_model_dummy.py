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
from std_msgs.msg import String
from std_msgs.msg import Int32
from nao_tutoring_behaviors.msg import TabletMsg
from nao_tutoring_behaviors.msg import ControlMsg

## This is a dummy model. It suggests tutoring behavoirs determinisitically based on the number of the question
## The relevant parts here are just those that deal with construction of model messages

class TutoringModel:
	def __init__(self):
		self.current_question = 1
		self.level = 1

		self.pid = -1
		self.sessionNum = -1
		self.expGroup = -1
		self.logFile = None

		self.tries = 0

		rospy.init_node('dummy_model', anonymous = True)
		self.decisons_pub = rospy.Publisher('model_decision_msg', ControlMsg, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz

		rospy.Subscriber("tablet_msg", TabletMsg, self.tablet_msg_callback) 	# subscribe to messages from the tablet node
		rospy.Subscriber("robot_speech_msg", String, self.robot_msg_callback)	# subscribe to messages from the robot

		# import all question jsons. Note: the tablet app should have access to the same files
		level_one_questions = []
		with open('/root/catkin_ws/src/nao_tutoring_behaviors/scripts/data/level1.json', 'r') as question_file_1:
			level_one_questions = json.load(question_file_1)
		level_two_questions = []
		with open('/root/catkin_ws/src/nao_tutoring_behaviors/scripts/data/level2.json', 'r') as question_file_2:
			level_two_questions = json.load(question_file_2)
		level_three_questions = []
		with open('/root/catkin_ws/src/nao_tutoring_behaviors/scripts/data/level3.json', 'r') as question_file_3:
			level_three_questions = json.load(question_file_3)
		with open('/root/catkin_ws/src/nao_tutoring_behaviors/scripts/data/level4.json', 'r') as question_file_4:
			level_four_questions = json.load(question_file_4)
		with open('/root/catkin_ws/src/nao_tutoring_behaviors/scripts/data/level5.json', 'r') as question_file_5:
			level_five_questions = json.load(question_file_5)
		
		self.questions = [[], level_one_questions, level_two_questions, level_three_questions, level_four_questions, level_five_questions]

	def repeat_question(self):									# send this message to have the student try the same question again
		control_message = ControlMsg()							# with no tutoring behavior
		control_message.nextStep = "QUESTION-REPEAT"
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = "Try that again."

		self.decisons_pub.publish(control_message)
		print "sent:" , control_message
		

	def next_question(self):									# indicates that the student should move on to the next question
		self.tries = 0	 	# reset the number of attempts on the question										
		self.current_question += 1 								# whether after a correct response or after enough incorrect attempts
																# without a message, the tablet will not display a new question
		if (self.current_question >= len(self.questions[self.level])):		
			self.question_next_level()

		control_message = ControlMsg()
		control_message.nextStep = "QUESTION-NEXT"
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']  # we give the text for this question 
																											# to the robot here
		print self.level, self.current_question, self.questions[self.level][self.current_question]
		
	
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message

	def question_next_level(self):								# indicates that the student should move to the next level
		self.tries = 0 											# so we increase the level number and go to question 1
		self.current_question = 1
		self.level += 1

		control_message = ControlMsg()
		control_message.nextStep = "QUESTION-LEVEL"
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']	# give the text for the question to the robot
	
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message


	def tic_tac_toe_break(self): 								# trigger a game of tic tac toe for a break
		control_message = ControlMsg() 							# the robot message here is the speech for the beginning of the game
		control_message.nextStep = "TICTACTOE"
		control_message.robotSpeech = "Lets play a game of tic-tac-toe. You will be exes, and I will be ohs. You can go first. Click any square on the board."
		
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message


	# def give_hint(self):										# this behavior does not exist anymore, but if desired, would give a text hint
	# 	control_message = ControlMsg()							# related to the problem (e.g. "how many times does the denominator go into 
	# 	control_message.nextStep = "SHOWHINT"	 				# the first digit of the numerator?"). The hint would be read in from some
	# 	control_message.questionNum = self.current_question		# list in node_tablet.py code 
	# 	control_message.questionLevel = self.level
	# 	control_message.robotSpeech = "Let me give you a hint"
		
	# 	self.decisons_pub.publish(control_message)
	# 	print "sent:" , control_message

	def give_structure_hint(self): 								# give a hint in the form of the structure of the problem
		control_message = ControlMsg()
		control_message.nextStep = "SHOWSTRUCTURE"
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = "Let's look at how to structure a solution"
		
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message

	def give_think_aloud(self):									# ask the student to think aloud
		control_message = ControlMsg()
		control_message.nextStep = "THINKALOUD"
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = "What is the first thing you want to do to solve this problem?"   # give the robot text here because node_tablet 
																										# doesn't do anything with this
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message


	def give_example(self):										# give a worked example - what example is given is determined by node_tablet code
		control_message = ControlMsg()							# and is based on the level. node_tablet controls all tablet and robot actions
		control_message.nextStep = "SHOWEXAMPLE"				# during this
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = ""
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message

	def give_tutorial(self):									# give an interactive tutorial. like the worked example, this is controled by node_tablet
		control_message = ControlMsg()
		control_message.nextStep = "SHOWTUTORIAL"
		control_message.questionNum = self.current_question
		control_message.questionLevel = self.level
		control_message.robotSpeech = ""
		self.decisons_pub.publish(control_message)
		print "sent:" , control_message		

	def tablet_msg_callback(self, data):										# respond to tablet messages by triggering the next behavior
		rospy.loginfo(rospy.get_caller_id() + "From Tablet, I heard %s", data)	# the code here is just based off the question number, but
																				# the real model can similarly respond to whether or not the 
		if (data.msgType == 'CA'): # respond to correct answer					# answer was correct and call one of these functions to produce
			self.next_question()												# a behavior
		elif (data.msgType == 'IA'): # respond to incorrect answer
			if (self.tries < 1):
				self.repeat_question()
				self.tries +=1
				print self.tries
			elif (data.questionNumOrPart % 4 == 0):
				#self.tic_tac_toe_break()
				#self.give_think_aloud()
				self.give_tutorial()

			elif (data.questionNumOrPart % 4 == 1):
				self.give_example()
			elif (data.questionNumOrPart % 4 == 2):
				self.give_structure_hint()
				#self.give_tutorial()
			else:
				self.give_structure_hint()
		elif (data.msgType == "TICTACTOE-WIN" or data.msgType == "TICTACTOE-LOSS"):	 # here I respond to the end of a game by going to the next
			self.next_question()													 # question, but you could return to the same one
		elif("SHOWEXAMPLE" in data.msgType):										 
			pass

	def robot_msg_callback(self, data):
		rospy.loginfo(rospy.get_caller_id() + "From Robot, I heard %s", data)		 # this model does nothing with robot messages, but it could
																					 # do so in this function

	def run(self):

		while not rospy.is_shutdown():
			try:
				pass

			except KeyboardInterrupt:
				self.logFile.flush()
				self.logFile.close()
				self.conn.close()
				self.store_session(self.current_session)
				sys.exit(0)



def run_model():
	model = TutoringModel()
	model.run()

if __name__ == '__main__':
    try:
        run_model()
    except rospy.ROSInterruptException:
        pass
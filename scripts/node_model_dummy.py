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
import bisect
import numpy as np
rospack = rospkg.RosPack()
from std_msgs.msg import String
from std_msgs.msg import Int32
from nao_tutoring_behaviors.msg import TabletMsg
from nao_tutoring_behaviors.msg import ControlMsg

sys.path.append(rospack.get_path('nao_tutoring_behaviors')+"/scripts/tutoring_action_selection")
from task_models.lib.pomdp import POMDP, GraphPolicyBeliefRunner
from pomdp_setup_observation_matrices import *
from pomdp_setup_reward_transition_matrices import *

## This is a dummy model. It suggests tutoring behavoirs determinisitically based on the number of the question
## The relevant parts here are just those that deal with construction of model messages

class TutoringModel:
    def __init__(self):
        self.total_num_questions = 0
        self.current_question = 0
        self.level = 1

        self.pid = -1
        self.sessionNum = -1
        self.expGroup = -1  #0 is control (fixed policy), 1 is our experimental condition (pomdp policy)
        self.difficultyGroup = -1
        self.logFile = None

        self.tries = 0

        self.attempt_times = []
        #self.total_num_help_actions = 0
        self.fixed_help_index = 0

        self.initial_knowledge_state = ""

        self.inSession = False

        #placeholder to hold pomdp model variables: current action, belief runner, etc --> initialize during START msg

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


    def setup_pomdp(self):
        param_file = rospack.get_path('nao_tutoring_behaviors')+"/scripts/data/03_13_B.json" #the param file that "works" for the base model
        with open(param_file) as data_file:
            params = json.load(data_file)

        # discount factor
        self.discount = params["discount"]

        # state variables
        knowledge_states = params["knowledge_states"]
        engagement_states = params["engagement_states"]
        attempt_states = params["attempt_states"]
        self.num_knowledge_levels = len(knowledge_states)
        self.num_engagement_levels = len(engagement_states)
        self.num_attempts = len(attempt_states)
        self.all_states = combine_states_to_one_list(knowledge_states, engagement_states, attempt_states)
        self.num_states = len(self.all_states)

        # starting distribution 
        start = np.zeros(self.num_states)
        if self.sessionNum ==1:
            initial_state = int(self.initial_knowledge_state[1:])
            majority_start = 0.7
            minority_start = (1.0 - majority_start) / (self.num_knowledge_levels-1)
            for i in range(self.num_knowledge_levels):
                #start[4 + i * 8] = 1.0 / float(self.num_knowledge_levels) #uniform start state
                if i==initial_state:
                    start[4 + i * 8] = majority_start
                else:
                    start[4 + i * 8] = minority_start

            self.action_prob_knowledge_gain_mult = params["action_prob_knowledge_gain_mult"]
            self.action_prob_engagement_gain_mult = params["action_prob_engagement_gain_mult"]        

        else:
            start = self.current_belief
            self.action_prob_knowledge_gain_mult = self.action_prob_knowledge_gain_mult #should be set earlier
            self.action_prob_engagement_gain_mult = self.action_prob_engagement_gain_mult

        # probabilities associated with the transition matrix
        self.prob_knowledge_gain = params["prob_knowledge_gain"]
        self.prob_engagement_gain = params["prob_engagement_gain"]
        self.prob_engagement_loss = params["prob_engagement_loss"]
        self.prob_correct_answer = params["prob_correct_answer"]
        self.prob_correct_answer_after_1_attempt = params["prob_correct_answer_after_1_attempt"]
        self.prob_drop_for_low_engagement = params["prob_drop_for_low_engagement"]

        # actions
        self.actions = params["actions"]
        self.num_actions = len(self.actions)

        # action-related reward variables
        action_rewards = params["action_rewards"]
        engagement_reward = params["engagement_reward"]
        knowledge_reward = params["knowledge_reward"]
        end_state_remain_reward = params["end_state_remain_reward"]
        reward_for_first_attempt_actions = params["reward_for_first_attempt_actions"]
        #action_prob_knowledge_gain_mult = params["action_prob_knowledge_gain_mult"]
        #self.action_prob_engagement_gain_mult = params["action_prob_engagement_gain_mult"]

        # observations
        correctness_obs = params["correctness_obs"]
        speed_obs = params["speed_obs"]
        self.all_obs = combine_obs_types_to_one_list(correctness_obs, speed_obs)
        self.num_observations = len(self.all_obs)

        # observation related variables
        self.prob_speeds_for_low_engagement = params["prob_speeds_for_low_engagement"]
        self.prob_speeds_for_high_engagement = params["prob_speeds_for_high_engagement"]
        action_speed_multipliers = np.array(params["action_speed_multipliers"])


        self.R = generate_reward_matrix(actions=self.actions,
                                   action_rewards=action_rewards, 
                                   engagement_reward=engagement_reward, 
                                   knowledge_reward=knowledge_reward, 
                                   end_state_remain_reward=end_state_remain_reward,
                                   num_knowledge_levels=self.num_knowledge_levels, 
                                   num_engagement_levels=self.num_engagement_levels,
                                   num_attempts=self.num_attempts, 
                                   num_observations=self.num_observations, 
                                   reward_for_first_attempt_actions=reward_for_first_attempt_actions)

        self.T = generate_transition_matrix(num_knowledge_levels=self.num_knowledge_levels, 
                                       num_engagement_levels=self.num_engagement_levels,
                                       num_attempts=self.num_attempts,
                                       prob_knowledge_gain=self.prob_knowledge_gain,
                                       prob_engagement_gain=self.prob_engagement_gain,
                                       prob_engagement_loss=self.prob_engagement_loss,
                                       action_prob_knowledge_gain_mult=self.action_prob_knowledge_gain_mult,
                                       action_prob_engagement_gain_mult=self.action_prob_engagement_gain_mult,
                                       prob_correct_answer=self.prob_correct_answer,
                                       prob_correct_answer_after_1_attempt=self.prob_correct_answer_after_1_attempt, 
                                       prob_drop_for_low_engagement=self.prob_drop_for_low_engagement)

        self.O = generate_observation_matrix(knowledge_states=knowledge_states, 
                                        engagement_states=engagement_states,
                                        attempt_states=attempt_states,
                                        correctness_obs=correctness_obs,
                                        speed_obs=speed_obs,
                                        num_actions=self.num_actions,
                                        prob_speeds_for_low_engagement=self.prob_speeds_for_low_engagement,
                                        prob_speeds_for_high_engagement=self.prob_speeds_for_high_engagement,
                                        action_speed_multipliers=action_speed_multipliers)


        #create POMDP model
        simple_pomdp = POMDP(self.T, self.O, self.R, np.array(start), self.discount, states=self.all_states, actions=self.actions,
                     observations=self.all_obs, values='reward')

        self.simple_pomdp_graph_policy = simple_pomdp.solve(method='grid', verbose=False, n_iterations=500)

        self.simple_pomdp_graph_policy_belief_runner = GraphPolicyBeliefRunner(self.simple_pomdp_graph_policy,
                                                                      simple_pomdp)

        self.current_belief = self.simple_pomdp_graph_policy_belief_runner.current_belief #should be the same as start?
        print "current belief is: "
        print self.current_belief
        self.action = self.simple_pomdp_graph_policy_belief_runner.get_action() #choose first action


    def resolve_pomdp(self):
        start = self.current_belief

        action_prob_knowledge_gain_mult = self.action_prob_knowledge_gain_mult


        self.T = generate_transition_matrix(num_knowledge_levels=self.num_knowledge_levels, 
                                       num_engagement_levels=self.num_engagement_levels,
                                       num_attempts=self.num_attempts,
                                       prob_knowledge_gain=self.prob_knowledge_gain,
                                       prob_engagement_gain=self.prob_engagement_gain,
                                       prob_engagement_loss=self.prob_engagement_loss,
                                       action_prob_knowledge_gain_mult=self.action_prob_knowledge_gain_mult,
                                       action_prob_engagement_gain_mult=self.action_prob_engagement_gain_mult,
                                       prob_correct_answer=self.prob_correct_answer,
                                       prob_correct_answer_after_1_attempt=self.prob_correct_answer_after_1_attempt, 
                                       prob_drop_for_low_engagement=self.prob_drop_for_low_engagement)

        simple_pomdp = POMDP(self.T, self.O, self.R, np.array(start), self.discount, states=self.all_states, actions=self.actions,
                     observations=self.all_obs, values='reward')

        self.simple_pomdp_graph_policy = simple_pomdp.solve(method='grid', verbose=False, n_iterations=500)

        self.simple_pomdp_graph_policy_belief_runner = GraphPolicyBeliefRunner(self.simple_pomdp_graph_policy,
                                                                      simple_pomdp)

        self.current_belief = self.simple_pomdp_graph_policy_belief_runner.current_belief #should be the same as start?
        print "current belief is: "
        print self.current_belief
        self.action = self.simple_pomdp_graph_policy_belief_runner.get_action() #choose first action
    
    
    def get_new_multipliers(self, obs, action):
        print "returning existing multipliers that are not changing for now"
        return self.action_prob_knowledge_gain_mult, self.action_prob_engagement_gain_mult


    def repeat_question(self):                                  # send this message to have the student try the same question again
        control_message = ControlMsg()                          # with no tutoring behavior
        control_message.nextStep = "QUESTION-REPEAT"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""#Try that again." #we don't want to say anything here.

        self.decisons_pub.publish(control_message)
        print "sent:" , control_message
        
    def set_up_session(self):
        control_message = ControlMsg()
        control_message.nextStep = "SETUP-SESSION"
        control_message.otherInfo = str(self.expGroup)

        #self.decisons_pub.publish(control_message)
        #print "sent: ", control_message


    def first_question(self):
        self.total_num_questions += 1
        self.tries = 0
        control_message = ControlMsg()
        control_message.nextStep = "QUESTION-FIRST"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']

        print self.level, self.current_question, self.questions[self.level][self.current_question]
        
        #time.sleep(3) #wait a little before sending first question message so robot can finish intro
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    
    def next_question(self):                                    # indicates that the student should move on to the next question
        #self.total_num_questions += 1                           # keeps track of the total number of questions student has seen 
        self.tries = 0                                          # reset the number of attempts on the question                                      
        self.level = self.total_num_questions % 3
        if self.difficultyGroup==0:
            self.level += 1
        else:
            self.level += 3
                                                                
        if self.total_num_questions % 3==0:
            self.current_question += 1

        self.total_num_questions += 1

        if (self.current_question >= len(self.questions[self.level])):
            print "this should only happen if student has really finished all questions"        
            #return self.question_next_level()

        control_message = ControlMsg()
        control_message.nextStep = "QUESTION-NEXT"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        if (self.current_question < len(self.questions[self.level])):
            control_message.robotSpeech = self.questions[self.level][self.current_question]['Spoken Question']  # we give the text for this question  to the robot here
        else:
            control_message.robotSpeech = "" #no speech in case they finish all questions
        #print self.level, self.current_question, self.questions[self.level][self.current_question]
        
        time.sleep(3) #wait a little before sending next question message so robot can say correct/incorrect
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def question_next_level(self): #aditi - no longer using         # indicates that the student should move to the next level
        self.tries = 0                                              # so we increase the level number and go to question 1
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
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = "Lets take a break and play a game of tic-tac-toe. You will be exes, and I will be ohs. You can go first. Click any square on the board."
        
        question_id = self.questions[self.level][self.current_question]['QuestionID']
        self.log_transaction("TICTACTOE-START", question_id, self.level)
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message


    def give_hint(self):                                        
        control_message = ControlMsg()                          
        control_message.nextStep = "SHOWHINT"                   
        control_message.questionNum = self.current_question     
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""
        
        question_id = self.questions[self.level][self.current_question]['QuestionID']
        self.log_transaction("HINT", question_id, self.level)
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
        control_message.robotSpeech = ""#"What is the first thing you want to do to solve this problem?" 
                                                                                                        
        question_id = self.questions[self.level][self.current_question]['QuestionID']
        self.log_transaction("THINKALOUD", question_id, self.level)
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message


    def give_example(self):                                     # give a worked example - what example is given is determined by node_tablet code
        control_message = ControlMsg()                          # and is based on the level. node_tablet controls all tablet and robot actions
        control_message.nextStep = "SHOWEXAMPLE"                # during this
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""
        
        question_id = self.questions[self.level][self.current_question]['QuestionID']
        self.log_transaction("WORKED-EXAMPLE", question_id, self.level)
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message

    def give_tutorial(self):                                    # give an interactive tutorial. like the worked example, this is controled by node_tablet
        control_message = ControlMsg()
        control_message.nextStep = "SHOWTUTORIAL"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""
        
        question_id = self.questions[self.level][self.current_question]['QuestionID']
        self.log_transaction("INTERACTIVE-TUTORIAL", question_id, self.level)
        self.decisons_pub.publish(control_message)
        print "sent:" , control_message     


    def no_action(self):
        control_message = ControlMsg()
        control_message.nextStep = "NOACTION"
        control_message.questionNum = self.current_question
        control_message.questionLevel = self.level
        control_message.robotSpeech = ""

        question_id = self.questions[self.level][self.current_question]['QuestionID']
        self.log_transaction("NO-ACTION", question_id, self.level)
        self.decisions_pub.publish(control_message)
        print "sent: ", control_message
    
    def get_mean_and_std_time(self):
        num_attempts_in_list = len(self.attempt_times)
        if num_attempts_in_list > 10:
            cutoff = int(.1 * num_attempts_in_list)
            print "cutoff is: " + str(cutoff)
        else:
            cutoff = 1

        front_index = cutoff
        back_index = num_attempts_in_list - cutoff
        mean = np.mean(self.attempt_times[front_index:back_index])
        std = np.std(self.attempt_times[front_index:back_index])
        print "mean is: " + str(mean)
        print "std is: " + str(std)
        return mean, std     


    def form_observation(self, msgType, timing):
        obs = ""
        if msgType == 'CA':
            obs += "R-"
        elif msgType == 'IA':
            obs += "W-"
        else:
            print "should not be here"
            return ""

        timing = float(timing) / 1000.0
        print "timing in seconds is: " + str(timing)

        if len(self.attempt_times) < 10: #changed to 10 so that we get 10 data points before calculating fast or slow
            obs += "med"

        else:
            mean, std = self.get_mean_and_std_time()

            zscore = float((timing - mean)) / float(std)
            print "zscore is: " + str(zscore)

            if zscore > 2.0:
                obs += "slow"
            elif zscore < -1.0:
                obs += "fast"
            else:
                obs += "med"

        return obs

    def add_attempt_time(self, timing):
        print "use this method to add timing to list of times"
        timing = float(timing) / 1000.0 #convert time in milliseconds to seconds
        if len(self.attempt_times) < 5:
            self.attempt_times.append(timing)
            self.attempt_times = sorted(self.attempt_times)
        else:
            bisect.insort(self.attempt_times, timing)
        print self.attempt_times    

    def log_transaction(self,msgType,questionID,otherInfo):
        transaction = str(self.pid) + "," + str(self.expGroup) + "," + str(self.sessionNum) + ","
        #transaction += str(datetime.datetime.now()) + ","
        transaction += str(int(round(time.time() * 1000))) + ","
        transaction += str(questionID) + ","
        transaction += msgType + ","
        transaction += str(otherInfo)  # put the level here for msgType==QUESTION
        self.logFile.write(transaction+"\n")
        self.logFile.flush()

    def log_multipliers(self, msgType, questionID, multipliers):
        transaction = str(self.pid) + "," + str(self.expGroup) + "," + str(self.sessionNum) + ","
        transaction += str(int(round(time.time() * 1000))) + ","
        transaction += str(questionID) + ","
        transaction += msgType + ","
        transaction += str(multipliers)
        self.logFile.write(transaction+"\n")
        self.logFile.flush()
    
    def tablet_msg_callback(self, data):                                            # respond to tablet messages by triggering the next behavior
        rospy.loginfo(rospy.get_caller_id() + " From Tablet, I heard %s ", data)    # the code here is just based off the question number, but
                                                                                    # the real model can similarly respond to whether or not the 
                                                                                    # answer was correct and call one of these functions to produce a behavior
        

        #first check if it was a correct or incorrect answer --> do the same thing for both
        #with any attempt, we need to get the obs then update the model. then we check msgType
        #again and go to next question if correct and provide help-action from model if incorrect
        if (data.msgType == 'CA' or data.msgType == 'IA'):
            question_id = self.questions[self.level][self.current_question]['QuestionID']
            attempt = data.otherInfo.split("-")[0]
            timing = int(data.otherInfo.split("-")[1])
            take_break = data.otherInfo.split("-")[2]
            observation = self.form_observation(data.msgType, timing)
            self.log_transaction("OBSERVATION", question_id, observation)
            print "observation is: " + str(observation)
            #placeholder to update belief using action that was just given and this observation
            self.add_attempt_time(timing)
            #placeholder to potentially sleep here if we want model to wait a few seconds before giving help

            if self.expGroup==1:
                self.current_belief = self.simple_pomdp_graph_policy_belief_runner.step(observation)
                self.current_belief = self.simple_pomdp_graph_policy_belief_runner.current_belief
                print "current belief is: " 
                print self.current_belief
                #before we get the next action, lets change our action multipliers and re-solve the pomdp
                self.action = self.simple_pomdp_graph_policy_belief_runner.get_action()
                #self.action_prob_knowledge_gain_mult, self.action_prob_engagement_gain_mult = self.get_new_multipliers(observation, self.action)
                #self.log_multipliers("KNOWLEDGE-MULT", question_id, self.action_prob_knowledge_gain_mult)
                #self.log_multipliers("ENGAGEMENT-MULT", question_id, self.action_prob_engagement_gain_mult)                


        if (data.msgType == 'CA'): # respond to correct answer
            self.fixed_help_index = 0
            attempt = data.otherInfo.split("-")[0]
            timing = int(data.otherInfo.split("-")[1])                  
            self.log_transaction("CORRECT", question_id, str(attempt)+"-"+str(timing))
            time.sleep(2)
            self.next_question()
            #self.resolve_pomdp()
            #self.action = self.simple_pomdp_graph_policy_belief_runner.get_action()                                                
        
        elif (data.msgType == 'IA'): # respond to incorrect answer
            attempt = data.otherInfo.split("-")[0]
            timing = int(data.otherInfo.split("-")[1])
            take_break = data.otherInfo.split("-")[2]
            self.log_transaction("INCORRECT", question_id, str(attempt)+"-"+str(timing))
            self.tries +=1

            #placeholder to get action from model then execute that action
            #check what the action is. then check experimental condition.
            #for control, log the model's action and execute action from fixed policy
            #for experimental, log the model's action, then do it.
            if self.expGroup==1:
                if(self.tries >= 3):
                    time.sleep(2)
                    self.next_question()
                    #self.resolve_pomdp()
                    #self.action = self.simple_pomdp_graph_policy_belief_runner.get_action()
                
                else:
                    
                    #self.resolve_pomdp()
                    #self.action = self.simple_pomdp_graph_policy_belief_runner.get_action()
                    print "DURING QUESTION, model will give this action: " + str(self.action)
                    #below, trying a time.sleep(2) because there is no lag when we dont resolve the pomdp.
                    time.sleep(2) #lets try not sleeping here for MODEL GROUP since we have the pomdp resolve time lag
                    if self.action=="no-action":
                        self.no_action()
                    elif self.action=="interactive-tutorial":
                        self.give_tutorial()
                    elif self.action=="worked-example":
                        self.give_example()
                    elif self.action=="hint":
                        self.give_hint()
                    elif self.action=="think-aloud":
                        self.give_think_aloud()
                    elif self.action=="break":
                        self.tic_tac_toe_break()
                    else:
                        print "error: model choosing action not in list"

            

            else: #this is the block that executes for the fixed group.

                if(self.tries >= 3):
                    time.sleep(2)
                    self.next_question()
                
                else:
                    #self.give_tutorial()
                    #self.give_think_aloud()
                    #self.give_hint()
                    time.sleep(5) #lets wait a little before giving help
                    
                    if self.expGroup==0: #implement fixed policy
                        if take_break=="takebreak":
                            self.tic_tac_toe_break()

                        else:
                            if self.fixed_help_index == 0:
                                self.give_think_aloud()
                            elif self.fixed_help_index == 1:
                                self.give_hint()
                            elif self.fixed_help_index == 2:
                                self.give_example()
                            elif self.fixed_help_index >= 3:
                                self.give_tutorial()
                            else:
                                print "should not be happening"
                            self.fixed_help_index += 1


                    else: #placeholder action selection for actual model
                        num = random.randint(0, 4)
                        time.sleep(3) # let's wait a little before starting any help activity
                        if num==0:
                            self.tic_tac_toe_break()
                        elif num==1:
                            self.give_tutorial()
                        elif num==2:
                            self.give_example()
                        elif num==3:
                            self.give_hint()
                        else:
                            self.give_think_aloud()

                

        elif (data.msgType == "TICTACTOE-END"): # here I respond to the end of a game by going to the same question
        #elif (data.msgType == "TICTACTOE-WIN" or data.msgType == "TICTACTOE-LOSS"):
            self.log_transaction("TICTACTOE-END", -1, "") 
            self.repeat_question()                                                
        
        elif ("SHOWEXAMPLE" in data.msgType):                                         
            pass
        
        elif ('SHOWING-QUESTION' in data.msgType):
            question_id = self.questions[self.level][self.current_question]['QuestionID']
            self.log_transaction("QUESTION", question_id, self.level)
            #placeholder to get current action on first attempt (should be no-action)
            if self.expGroup==1:
                #self.action = self.simple_pomdp_graph_policy_belief_runner.get_action()
                if self.tries==0:
                    print "START OF NEW QUESTION, MODEL CHOSE ACTION: " + str(self.action)
                    if self.action!="no-action":
                        if self.action=="interactive-tutorial":
                            self.give_tutorial()
                        elif self.action=="worked-example":
                            self.give_example()
                        elif self.action=="hint":
                            self.give_hint()
                        elif self.action=="think-aloud":
                            self.give_think_aloud()
                        elif self.action=="break":
                            self.tic_tac_toe_break()
                else:
                    print "MIDDLE OF A QUESTION, so not actually doing action here: " + str(self.action)
                
        elif (data.msgType == 'START' or data.msgType == 'LOAD'):
            print "MODEL RECEIVED START MESSAGE FROM TABLET_MSG --------------> setting up session"
            self.inSession = True
            self.pid = int(data.questionNumOrPart)
            self.sessionNum = int(data.questionType)
            self.expGroup = int(data.robotSpeech)
            print "EXPGROUP IS: " + str(self.expGroup)
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
                self.total_num_questions = 0
                if self.difficultyGroup == 1:
                    print "harder difficulty group"
                    self.questions = self.harder_questions
                    self.level = 3
                else:
                    print "easier difficulty group"
                    self.level = 1
                
                saveFileString = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/" + "P"+str(self.pid)+"_save.json"
                if os.path.exists(saveFileString): #only if this file already exists are we loading a session that crashed
                    with open(saveFileString) as param_file:
                        params = json.load(param_file)

                    self.expGroup = int(params["expGroup"])
                    self.difficultyGroup = int(params["difficultyGroup"])
                    num_problems = int(params["numProblemsCompleted"])
                    self.total_num_questions = num_problems #this tracks total number of q's over all sessions
                    self.attempt_times = params["attemptTimes"]
                    self.fixed_help_index = params["fixedHelpIndex"]
                    #self.current_question = params["currentQuestionIndex"]
                    if self.difficultyGroup == 1:
                        self.questions = self.harder_questions
                        self.level = (num_problems % 3) + 3
                    else:
                        self.level = (num_problems % 3) + 1
                    self.current_question = num_problems / 3

                    if self.expGroup==1:
                        self.current_belief = np.array(params["currentBelief"])
                        self.action_prob_knowledge_gain_mult = params["action_prob_knowledge_gain_mult"]
                        self.action_prob_engagement_gain_mult = params["action_prob_engagement_gain_mult"]

                else: #the param_save file does not exist so it is a new session.
                    if self.expGroup==1: #if expGroup 1, read the knowledge_start_state file to choose start dist for pomdp
                        startStateFile = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/initial_knowledge_states.json"
                        if os.path.exists(startStateFile):
                            with open(startStateFile) as start_state_file:
                                start_states = json.load(start_state_file)

                            self.initial_knowledge_state = start_states[str(self.pid)]

            
            else: #later sessions after session 1
                self.attempt_times = []
                saveFileString = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/" + "P"+str(self.pid)+"_save.json"
                if os.path.exists(saveFileString):
                    with open(saveFileString) as param_file:
                        params = json.load(param_file)

                    self.expGroup = int(params["expGroup"])
                    self.difficultyGroup = int(params["difficultyGroup"])
                    num_problems = int(params["numProblemsCompleted"])
                    self.total_num_questions = num_problems #this tracks total number of q's over all sessions
                    self.attempt_times = params["attemptTimes"]
                    self.fixed_help_index = params["fixedHelpIndex"]
                    #self.current_question = params["currentQuestionIndex"]
                    if self.difficultyGroup == 1:
                        self.questions = self.harder_questions
                        self.level = (num_problems % 3) + 3
                    else:
                        self.level = (num_problems % 3) + 1
                    self.current_question = num_problems / 3

                    if self.expGroup==1:
                        self.current_belief = np.array(params["currentBelief"])
                        self.action_prob_knowledge_gain_mult = params["action_prob_knowledge_gain_mult"]
                        self.action_prob_engagement_gain_mult = params["action_prob_engagement_gain_mult"]
                else:
                    print "error: tried to open param save file when it didnt exist"
            #self.send_first_question()
            #time.sleep(3) #wait a bit before sending first question - do we need this?
            #self.first_question()
            #self.next_question() #aditi - trying this instead since send_first_question does not exist
            if self.expGroup==1:
                self.setup_pomdp()
        
        elif(data.msgType == 'END'):
            self.inSession = False
            print "End of session - should try to save whatever info is needed to restart for next session"
            self.save_params()
            self.log_transaction("END", -1, "")
            self.logFile.flush()
            self.logFile.close()
            


    def save_params(self):
        saveFileString = rospack.get_path('nao_tutoring_behaviors')+"/scripts/logfiles/" + "P"+str(self.pid)+"_save.json"
        self.save_file = open(saveFileString, "w+")
        num_problems_completed = self.total_num_questions
        save_help_index = self.fixed_help_index
        if self.fixed_help_index > 0:
            save_help_index = self.fixed_help_index - 1
        
        save_params = {"expGroup": self.expGroup,
                       "difficultyGroup": self.difficultyGroup,
                       "numProblemsCompleted": num_problems_completed,
                       "attemptTimes": self.attempt_times,
                       "fixedHelpIndex": save_help_index}
        if self.expGroup==1:
            print self.current_belief
            if self.tries!=0:
                #if we end on an attempt in the middle of a problem, we want to change the belief state to start back in A0 states
                for i in range(len(self.current_belief)):
                    if i%4==0:
                        if self.current_belief[i+2]!=0.0:
                            self.current_belief[i+1] = self.current_belief[i+2]
                            self.current_belief[i+2] = 0.0
                        elif self.current_belief[i+3]!=0:
                            self.current_belief[i+1] = self.current_belief[i+3]
                            self.current_belief[i+3] = 0.0
            print self.current_belief
            save_params["currentBelief"] = self.current_belief.tolist()
            save_params["action_prob_knowledge_gain_mult"] = self.action_prob_knowledge_gain_mult
            save_params["action_prob_engagement_gain_mult"] = self.action_prob_engagement_gain_mult
        param_string = json.dumps(save_params, indent=4)
        self.save_file.write(param_string)
        self.save_file.flush()
        self.save_file.close()

    

    def robot_msg_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " From Robot, I heard %s ", data)      # this model does nothing with robot messages, but it could
                                                                                     # do so in this function
        if (data.data == "INTRO-DONE"):
            self.first_question()

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

        if self.inSession:
            print "saving params because app is done"
            self.save_params()
            self.logFile.flush()
            self.logFile.close()



def run_model():
    model = TutoringModel()
    model.run()

if __name__ == '__main__':
    try:
        run_model()
    except rospy.ROSInterruptException:
        pass

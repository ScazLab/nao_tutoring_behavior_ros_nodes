import random
import time

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior

import almath
import animacyStrings as anim
BASEPATH = "/home/nao/behaviors/"


class Gesture:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.stiffness = 1.0

        self.frame = None
        self.speechDevice = None
        self.motion = None
        self.posture = None
        self.led = None
        self.right = anim.right
        self.wrong = anim.wrong
        self.wrong_postfix = anim.wrong_postfix
        self.trouble = anim.trouble
        self.hint = anim.hint
        self.confused = anim.confused
        self.auto_hint = anim.auto_hint
        self.question_intro = anim.question_intro
        self.question_intro_generic = anim.question_intro_generic
        self.connectNao()

        "Robot".__init__(self, "Robot")

    def connectNao(self):
        # FRAME MANAGER FOR CALLING BEHAVIORS
        try:
            print 'in connect nao, host and port are:', self.host, ',', self.port
            self.frame = ALProxy("ALFrameManager", self.host, self.port)
        except Exception, e:
            print "Error when creating frame manager device proxy:" + str(e)
            exit(1)
        # POSTURE MANAGER
        try:
            self.posture = postureProxy = ALProxy("ALRobotPosture", self.host, self.port)
        except Exception, e:
            print "Error creating posture proxy" + str(e)
            exit(1)

        # MOTION DEVICE FOR MOVEMENTS
        try:
            self.motion = ALProxy("ALMotion", self.host, self.port)
        except Exception, e:
            print "Error when creating motion device proxy:" + str(e)
            exit(1)

        # MAKE NAO STIFF (OTHERWISE IT WON'T MOVE)
        self.motion.stiffnessInterpolation("Body", self.stiffness, 1.0)

        # MOTION DEVICE FOR MOVEMENTS
        try:
            self.led = ALProxy("ALLeds", self.host, self.port)
        except Exception, e:
            print "Error when creating led proxy:" + str(e)
            exit(1)

        # CONNECT TO A SPEECH PROXY
        try:
            self.speechDevice = ALProxy("ALTextToSpeech", self.host, self.port)
        except Exception, e:
            print "Error when creating speech device proxy:" + str(e)
            exit(1)
        
        try:   
            self.animatedSpeech = ALProxy("ALAnimatedSpeech", self.host, self.port)
            self.memory = ALProxy("ALMemory", self.host, self.port)
            # self.speakMove = ALProxy("ALSpeakingMovement", self.host, self.port)
            self.configuration = {"bodyLanguageMode":"contextual"}
        except Exception, e:
            print "Error when creating animated Speech proxy"
            exit(1)

        # bring to sitting position
        self.posture.goToPosture("Sit", 0.15)
        self.motion.setBreathEnabled("Body",False)


    def animated_speech_if_non_empty(self, speech):
        self.motion.setBreathEnabled("Body",False)
        self.motion.stiffnessInterpolation("Body",self.stiffness,1.0)
        if (speech != ""):
            self.animatedSpeech.say(speech, self.configuration)

    def animated_speech_return_to_neutral(self, speech):
        self.motion.setBreathEnabled("Body",False)
        self.motion.stiffnessInterpolation("Body",self.stiffness,1.0)
        if (speech != ""):
            self.animatedSpeech.say(speech, self.configuration)
        self.bring_to_sit()

    def move_on_to_next_speech(self):
        phrases = ["Try the next one! Press the button on the screen!", "Use the button on the screen and lets go on to the next question!", "Press the next question button on the screen and let's move on!"]
        speech  = random.choice(phrases)
        self.animated_speech_return_to_neutral(speech)

    def correct_answer_speech(self):           
        phrases = ["Good job!", "Well done!", "Very good!", "Good answer!", "That's correct!"]
        speech  = random.choice(phrases)
        self.animated_speech_return_to_neutral(speech)


    def incorrect_answer_speech(self):            
        phrases = ["That's not quite right.", "Oh no. That's not right"]
        speech  = random.choice(phrases)
        self.animated_speech_return_to_neutral(speech)

    def genSpeech(self, sentence):
        try:
            # self.look()
            id = self.speechDevice.post.say(sentence)
            return id
        except Exception, e:
            print "Error when saying a sentence: " + str(e)

    def send_command(self, doBehavior):
        gesture_path = BASEPATH + doBehavior
        gesture_id = self.frame.newBehaviorFromFile(gesture_path, "")
        self.frame.playBehavior(gesture_id)
        self.frame.completeBehavior(gesture_id)

    def goodbye(self):
        self.genSpeech(anim.finish)
        time.sleep(5)
        self.bring_to_sit(1.0)

    def session_intro(self, sessionNum):
        # print sessionNum
        welcomePhrase = ''
        if sessionNum == 1:
            welcomePhrase = "Hello! My name is Nao. I am your personal robot tutor!"
        elif sessionNum == 2:
            welcomePhrase = "Welcome back!"
        elif sessionNum == 3:
            welcomePhrase = "Nice to see you again!"
        elif sessionNum == 4:
            welcomePhrase = "Hello! It is great to see you again!"
        elif sessionNum == 5:
            welcomePhrase = "Hello again! Today is our last session."

        id = self.genSpeech(welcomePhrase)
        self.look()
        self.wave()
        self.look()
        self.speechDevice.wait(id, 0)

        if sessionNum == 1:
            self.animatedSpeech.say("We will do some math problems together today.", self.configuration)
            self.animatedSpeech.say("I will put the math problems on the tablet in front of you.", self.configuration)
            id = self.animatedSpeech.say("Great! Let's get started!", self.configuration)
            self.bring_to_sit()
        elif sessionNum == 2:
            id = self.animatedSpeech.say("Lets get started on some more problems!", self.configuration)
            self.bring_to_sit()
        elif sessionNum == 3:
            id = self.animatedSpeech.say("Lets try doing some more problems today!", self.configuration)
            self.bring_to_sit()
        elif sessionNum == 4:
            id = self.animatedSpeech.say("Lets practice some more problems today!", self.configuration)
            self.bring_to_sit()
        elif sessionNum == 5:
            id = self.animatedSpeech.say("Lets jump into our last set of practice problems!", self.configuration)
            self.bring_to_sit()
        else:
            print "invalid sessionNum: no intro"
        # time.sleep(2)
        return id

    def session_goodbye(self, sessionNum):
        self.congratulations()
        id = self.genSpeech("Congratulations! You have completed the session.")
        self.speechDevice.wait(id, 0)

        if sessionNum == 1:
            id = self.animatedSpeech.say("I had a great time doing math with you! See you next time!")
        elif sessionNum == 2:
            id = self.animatedSpeech.say("It was fun working on math problems together today! See you soon!")
        elif sessionNum == 3:
            id = self.animatedSpeech.say("I had an awesome time doing math problems with you! See you next time!")
        elif sessionNum == 4:
            id = self.animatedSpeech.say("That was fun! See you again soon for our next session!")
        elif sessionNum == 5:
            id = self.animatedSpeech.say("It was great doing math with you! I hope you had fun too! Have a great day! Bye!")
        else:
            print "invalid sessionNum: no goodbye"

        self.bring_to_sit()
        return id



    def move_head(self):
        self.bring_to_sit(1.0)
        self.motion.setStiffnesses("Head", 1.0)
        # Example showing multiple trajectories
        # Interpolate the head yaw to 1.0 radian and back to zero in 2.0 seconds
        # while interpolating HeadPitch up and down over a longer period.
        # names  = ["HeadYaw","HeadPitch"]
        names = ["HeadPitch"]
        # Each joint can have lists of different lengths, but the number of
        # angles and the number of times must be the same for each joint.
        # Here, the second joint ("HeadPitch") has three angles, and
        # three corresponding times.
        # angleLists  = [[50.0*almath.TO_RAD, 0.0],
        #           [-30.0*almath.TO_RAD, 30.0*almath.TO_RAD, 0.0]]
        angleLists = [-10.0 * almath.TO_RAD, 20.0 * almath.TO_RAD, 0.0]
        # timeLists   = [[1.0, 2.0], [ 1.0, 2.0, 3.0]]
        timeLists = [1.0, 2.0, 3.0]
        isAbsolute = True

        self.genSpeech("hello my name is nao")
        for j in range(5):
             self.motion.angleInterpolation(names, angleLists, timeLists,
                                            isAbsolute)
    def wave(self):
        self.motion.setAngles("RShoulderPitch", -1.0, 0.15)
        self.motion.setAngles("RShoulderRoll", -1.2, 0.15)
        self.motion.setAngles("RElbowRoll", 1.0, 0.1)
        self.motion.setAngles("RElbowYaw", 0.5, 0.1)
        self.motion.setAngles("RWristYaw", 0, 0.1)
        self.motion.openHand("RHand")

        time.sleep(0.7)

        # wave the hand 3 times, by moving the elbow
        for  i in range(3):
            self.motion.setAngles("RElbowRoll", 1.5, 0.5)
            time.sleep(0.5)
            self.motion.setAngles("RElbowRoll", 0.5, 0.5)
            time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 1.0, 0.5)
        time.sleep(1)
        self.motion.closeHand("RHand")
        self.prepare_sit_right(0.15)
        time.sleep(4)
        self.bring_to_sit(1)

    def juddNelson(self):
        # this bit of code makes the robot thrust its right hand into the air
        self.bring_to_sit(0.5)
        time.sleep(0.1)
        # move to position
        self.motion.setAngles("RShoulderPitch", -1.0, 0.3)
        self.motion.setAngles("RShoulderRoll", -1.3, 0.3)
        self.motion.setAngles("RElbowRoll", 1.5, 0.3)
        self.motion.setAngles("RWristYaw", 0, 0.3)
        self.motion.closeHand("RHand")
        time.sleep(0.1)
        # time.sleep(1.0)
        # self.genSpeech("good job! that's correct!")
        self.motion.setAngles("RShoulderRoll", -1.0, 0.25)
        self.motion.setAngles("RElbowRoll", 1.0, 0.5)
        time.sleep(1.5)
        self.motion.setAngles("RShoulderRoll", -0.3, 0.15)
        self.motion.setAngles("RShoulderPitch", 1, 0.15)
        time.sleep(2)
        self.prepare_sit_right(0.15)

    def juddNelson_left(self):
        # this bit of code makes the robot thrust its left hand into the air
        # start in sitting position
        self.bring_to_sit(0.5)
        time.sleep(0.1)
        # move to position
        self.motion.setAngles("LShoulderPitch", -1.0, 0.3)
        self.motion.setAngles("LShoulderRoll", 1.3, 0.3)
        self.motion.setAngles("LElbowRoll", -1.5, 0.3)
        self.motion.setAngles("LWristYaw", 0, 0.3)
        self.motion.closeHand("LHand")
        time.sleep(0.1)
        # pump fist upward
        self.motion.setAngles("LShoulderRoll", 1.0, 0.25)
        self.motion.setAngles("LElbowRoll", -1.0, 0.5)

        time.sleep(1.5)
        self.motion.setAngles("LShoulderRoll", 0.3, 0.15)
        self.motion.setAngles("LShoulderPitch", 1, 0.15)
        time.sleep(2)
        self.prepare_sit_left(0.15)
        time.sleep(2)
        # return to sitting position
        self.bring_to_sit(1.0)

    def nod(self):
        # this bit of code makes the robot nod its head
        # self.posture.goToPosture("Sit", 0.5)
        self.motion.setStiffnesses("Head", 1.0)
        time.sleep(0.5)

        # move head
        # self.motion.setAngles("HeadPitch", 0, 0.5)
        # time.sleep(0.5)
        self.motion.setAngles("HeadPitch", 0.15, 0.25)
        time.sleep(0.5)
        self.motion.setAngles("HeadPitch", 0, 0.25)
        time.sleep(0.5)
        self.motion.setAngles("HeadPitch", 0.15, 0.25)

        # move back to original position
        time.sleep(3)
        self.bring_to_sit(1.0)

    def shake(self):
        # this bit of code makes the robot shake its head
        # self.posture.goToPosture("Sit", 0.5)
        self.motion.setStiffnesses("Head", 1.0)
        # time.sleep(0.5)

        # shake head
        self.motion.setAngles("HeadPitch", 0, 0.05)
        # self.motion.setAngles("HeadYaw", 0, 0.05)
        # time.sleep(0.5)
        self.motion.setAngles("HeadYaw", -0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", -0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0, 0.3)

        time.sleep(1)
        self.look()
        time.sleep(0.5)
        self.bring_to_sit(1.0)

    def last_shake(self):
        # this bit of code makes the robot shake its head
        # self.posture.goToPosture("Sit", 0.5)
        self.motion.setStiffnesses("Head", 1.0)
        time.sleep(0.5)

        # shake head
        self.motion.setAngles("HeadPitch", 0, 0.3)
        for i in range(2):
            time.sleep(0.5)
            self.motion.setAngles("HeadYaw", -0.5, 0.3)
            time.sleep(0.5)
            self.motion.setAngles("HeadYaw", 0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0, 0.3)

        # move back to the original position
        time.sleep(2.5)
        self.bring_to_sit(1.0)

    def scale_up(self):
        # start position of the arm
        self.motion.setAngles("RShoulderPitch", 0.4, 0.1)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.1)
        self.motion.setAngles("RElbowYaw", 1.5, 0.1)
        self.motion.setAngles("RElbowRoll", 0.4, 0.1)
        self.motion.setAngles("RWristYaw", 1.6, 0.1)
        self.motion.openHand("RHand")

        # end position of the arm
        self.motion.setAngles("RShoulderPitch", 0.4, 0.3)
        self.motion.setAngles("RElbowRoll", 1.3, 0.3)

        time.sleep(2)
        self.bring_to_sit(0.15)
        time.sleep(2)

    def scale_down(self):
        # start position of the arm
        self.motion.setAngles("RShoulderPitch", 0.7, 0.2)
        self.motion.setAngles("RShoulderRoll", -1.2, 0.2)
        self.motion.setAngles("RElbowYaw", 1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 1.4, 0.2)
        self.motion.setAngles("RWristYaw", 0, 0.2)
        self.motion.openHand("RHand")

        # end position of the arm
        self.motion.setAngles("RShoulderPitch", 1.5, 0.1)

        time.sleep(2.5)
        self.bring_to_sit(0.15)
        time.sleep(2)

    def scale_down_left(self):
        # start position of the arm
        self.motion.setAngles("LShoulderPitch", 0.7, 0.2)
        self.motion.setAngles("LShoulderRoll", 1.2, 0.2)
        self.motion.setAngles("LElbowYaw", -1.5, 0.2)
        self.motion.setAngles("LElbowRoll", -1.4, 0.2)
        self.motion.setAngles("LWristYaw", 0, 0.2)
        self.motion.openHand("LHand")

        # end position of the arm
        self.motion.setAngles("LShoulderPitch", 1.5, 0.1)

        time.sleep(2.5)
        self.prepare_sit_left(0.15)

        time.sleep(2)

    def two_fractions(self):
        # moving the right hand to the beginning position
        self.motion.setAngles("HeadPitch", 0, 0.15)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RShoulderPitch", 0.4, 0.2)
        self.motion.setAngles("RElbowRoll", 1.2, 0.2)
        self.motion.setAngles("RElbowYaw", 1.7, 0.2)
        self.motion.setAngles("RWristYaw", 1.5, 0.2)

        time.sleep(0.6)

        # emphasize the right hand
        self.motion.setAngles("HeadYaw", -0.7, 0.2)
        self.motion.setAngles("RElbowRoll", 0.2, 0.2)

        time.sleep(0.2)

        # moving the left hand to the beginning position
        self.motion.setAngles("LShoulderRoll", 0.5, 0.25)
        self.motion.setAngles("LShoulderPitch", 0.4, 0.25)
        self.motion.setAngles("LElbowRoll", -1.2, 0.25)
        self.motion.setAngles("LElbowYaw", -1.7, 0.25)
        self.motion.setAngles("LWristYaw", -1.5, 0.25)

        time.sleep(0.6)

        # emphasize the left hand
        self.motion.setAngles("HeadYaw", 0.7, 0.2)
        self.motion.setAngles("LElbowRoll", -0.2, 0.2)

        # open both hands and move the head back center
        self.motion.openHand("RHand")
        self.motion.setAngles("HeadYaw", 0, 0.05)
        self.motion.openHand("LHand")

        time.sleep(0.6)

        # bring arms in to avoid scooching sit
        self.motion.setAngles("RShoulderRoll", -0.15, 0.2)
        self.motion.setAngles("LShoulderRoll", 0.15, 0.2)
        self.motion.setAngles("RElbowYaw", 0, 0.2)
        self.motion.setAngles("LElbowYaw", 0, 0.2)
        self.motion.setAngles("LElbowRoll", -1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 1.5, 0.2)
        time.sleep(1)

        self.bring_to_sit(0.5)

    # used for word problems
    def two_hands(self):
        time.sleep(9)

        # moving the right hand to the beginning position
        self.motion.setAngles("HeadPitch", 0, 0.15)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RShoulderPitch", 0.4, 0.2)
        self.motion.setAngles("RElbowRoll", 1.2, 0.2)
        self.motion.setAngles("RElbowYaw", 1.7, 0.2)
        self.motion.setAngles("RWristYaw", 1.5, 0.2)

        # moving the left hand to the beginning position
        self.motion.setAngles("LShoulderRoll", 0.5, 0.25)
        self.motion.setAngles("LShoulderPitch", 0.4, 0.25)
        self.motion.setAngles("LElbowRoll", -1.2, 0.25)
        self.motion.setAngles("LElbowYaw", -1.7, 0.25)
        self.motion.setAngles("LWristYaw", -1.5, 0.25)

        time.sleep(0.8)

        # emphasize the right hand
        # self.motion.setAngles("HeadYaw", -0.7, 0.2)
        self.motion.setAngles("RElbowRoll", 0.2, 0.2)

        # time.sleep(0.3)

        # emphasize the left hand
        # self.motion.setAngles("HeadYaw", 0.7, 0.2)
        self.motion.setAngles("LElbowRoll", -0.2, 0.2)

        # open both hands and move the head back center
        self.motion.openHand("RHand")
        self.motion.setAngles("HeadYaw", 0, 0.05)
        self.motion.openHand("LHand")

        time.sleep(0.5)

        # bring arms in to avoid scooching sit
        self.motion.setAngles("RShoulderRoll", -0.15, 0.2)
        self.motion.setAngles("LShoulderRoll", 0.15, 0.2)
        self.motion.setAngles("RElbowYaw", 0, 0.2)
        self.motion.setAngles("LElbowYaw", 0, 0.2)
        self.motion.setAngles("LElbowRoll", -1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 1.5, 0.2)
        time.sleep(1)

        self.bring_to_sit(0.5)

    def multiples(self):
        # moving the right hand to the beginning position
        self.motion.setAngles("HeadPitch", 0, 0.15)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RShoulderPitch", 0.4, 0.2)
        self.motion.setAngles("RElbowRoll", 1.2, 0.2)
        self.motion.setAngles("RElbowYaw", 1.7, 0.2)
        self.motion.setAngles("RWristYaw", 1.5, 0.2)

        time.sleep(0.8)

        # emphasize the right hand
        self.motion.setAngles("HeadYaw", -0.7, 0.2)
        self.motion.setAngles("RElbowRoll", 0.2, 0.2)

        time.sleep(2.5)

        # moving the left hand to the beginning position
        self.motion.setAngles("LShoulderRoll", 0.5, 0.25)
        self.motion.setAngles("LShoulderPitch", 0.4, 0.25)
        self.motion.setAngles("LElbowRoll", -1.2, 0.25)
        self.motion.setAngles("LElbowYaw", -1.7, 0.25)
        self.motion.setAngles("LWristYaw", -1.5, 0.25)

        time.sleep(0.8)

        # emphasize the left hand
        self.motion.setAngles("HeadYaw", 0.7, 0.2)
        self.motion.setAngles("LElbowRoll", -0.2, 0.2)

        # wait to bring hands together
        # time.sleep(2)

        # open both hands and move the head back center
        self.motion.openHand("RHand")
        self.motion.setAngles("HeadYaw", 0, 0.05)
        self.motion.openHand("LHand")

        time.sleep(0.9)

        # bring arms in to avoid scooching sit
        self.motion.setAngles("RShoulderRoll", -0.15, 0.2)
        self.motion.setAngles("LShoulderRoll", 0.15, 0.2)
        self.motion.setAngles("RElbowYaw", 0, 0.2)
        self.motion.setAngles("LElbowYaw", 0, 0.2)
        self.motion.setAngles("LElbowRoll", -1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 1.5, 0.2)
        time.sleep(1)

        self.bring_to_sit(0.5)

    def look(self):
        # head looking at student and talking, for right handers
        self.motion.setAngles("HeadYaw", -0.25, 0.1)
        self.motion.setAngles("HeadPitch", 0.15, 0.1)

    def look_left(self):
        # head looking at student and talking, for left handers
        self.motion.setAngles("HeadYaw", 0.25, 0.1)
        self.motion.setAngles("HeadPitch", 0.15, 0.1)
    # this function doesn't actually sit, but gets called in nao_server if
    # introFlag is false, so is left in this file
    def sit(self):
        #self.posture.goToPosture("Sit", 0.2)
        #head looks down at tablet, for right handers
        self.motion.setAngles("HeadYaw", 0.3, 0.1)
        self.motion.setAngles("HeadPitch", 0.37, 0.1)
        #self.motion.setAngles("HeadYaw", 0.25, 0.15)
    def sit_left(self):
        # head looks down at tablet, for left handers
        self.motion.setAngles("HeadYaw", 0, 0.1)
        self.motion.setAngles("HeadPitch", 0.33, 0.1)

    def left_relaxed_sit(self):
        # move the left hand up and out of the way to avoid collision
        self.motion.setAngles("HeadPitch", 0.3, 0.15)
        self.motion.setAngles("LShoulderPitch", 0.5, 0.2)
        self.motion.setAngles("LShoulderRoll", 0.5, 0.2)
        self.motion.setAngles("LElbowRoll", -0.1, 0.2)

        time.sleep(1)

        # move hand around to the side
        self.motion.setAngles("LShoulderPitch", 1.8, 0.2)
        self.motion.setAngles("LShoulderRoll", 0.5, 0.2)

        time.sleep(0.5)

        # adjust elbow inward
        self.motion.setAngles("LElbowRoll", -1.1, 0.2)

        # adjust shoulder accordingly
        self.motion.setAngles("LShoulderRoll", 0.2, 0.2)

        time.sleep(5)

        # backtrack motions to prevent collision with leg
        self.motion.setAngles("LShoulderRoll", 0.5, 0.2)
        time.sleep(0.1)
        self.motion.setAngles("LElbowRoll", -0.1, 0.2)
        self.motion.setAngles("LShoulderPitch", 0.5, 0.2)

        time.sleep(0.5)

        self.bring_to_sit(0.5)
        self.motion.setAngles("HeadPitch", 0.3, 0.15)

    def right_relaxed_sit(self):
        # move the right hand up and out of the way to avoid collision
        self.motion.setAngles("HeadPitch", 0.3, 0.15)
        self.motion.setAngles("RShoulderPitch", 0.5, 0.2)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RElbowRoll", 0.1, 0.2)

        time.sleep(1)

        # move hand around to the side
        self.motion.setAngles("RShoulderPitch", 1.8, 0.2)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)

        time.sleep(0.5)

        # adjust elbow inward
        self.motion.setAngles("RElbowRoll", 1.1, 0.2)

        # adjust shoulder accordingly
        self.motion.setAngles("RShoulderRoll", -0.2, 0.2)

        time.sleep(5)

        # backtrack motions to prevent collision with leg
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        time.sleep(0.1)
        self.motion.setAngles("RElbowRoll", 0.1, 0.2)
        self.motion.setAngles("RShoulderPitch", 0.5, 0.2)

        time.sleep(0.5)

        self.bring_to_sit(0.5)
        self.motion.setAngles("HeadPitch", 0.3, 0.15)

    def numerator(self):
        self.bring_to_sit(1.0)

        # start position of the arm
        self.motion.setAngles("RShoulderPitch", -0.3, 0.2)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RElbowYaw", 1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 0.4, 0.2)
        self.motion.setAngles("RWristYaw", 1.6, 0.2)
        self.motion.openHand("RHand")

        time.sleep(2)
        self.prepare_sit_right(0.15)
        time.sleep(2)

    def denominator(self):
        self.bring_to_sit(1.0)

        # start position of the arm
        # self.motion.setAngles("HeadPitch", 0, 0.15)
        self.motion.setAngles("RShoulderPitch", 0.8, 0.2)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RElbowYaw", 1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 0.4, 0.2)
        self.motion.setAngles("RWristYaw", 1.6, 0.2)
        self.motion.openHand("RHand")

        time.sleep(2)

        # raise up elbow to avoid collision
        self.motion.setAngles("RElbowRoll", 0.8, 0.2)
        self.motion.setAngles("RWristYaw", 1.3, 0.2)

        self.prepare_sit_right(0.15)

    def numerator_denominator(self):
        self.bring_to_sit(1.0)

        time.sleep(2.5)

        self.motion.setAngles("RShoulderPitch", -0.3, 0.2)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RElbowYaw", 1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 0.4, 0.2)
        self.motion.setAngles("RWristYaw", 1.6, 0.2)
        self.motion.openHand("RHand")

        time.sleep(0.3)

        self.motion.setAngles("RShoulderPitch", 0.8, 0.2)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        self.motion.setAngles("RElbowYaw", 1.5, 0.2)
        self.motion.setAngles("RElbowRoll", 0.4, 0.2)
        self.motion.setAngles("RWristYaw", 1.6, 0.2)
        self.motion.openHand("RHand")

        time.sleep(0.5)
        self.prepare_sit_right(0.15)
        time.sleep(2)

    def etc(self):
        # head faces straight
        self.motion.setAngles("HeadPitch", 0, 0.15)

        time.sleep(0.25)

        # start position of the arm
        self.motion.setAngles("LShoulderRoll", 0.3, 0.25)
        self.motion.setAngles("LShoulderPitch", 0.4, 0.25)
        self.motion.setAngles("LElbowRoll", -1.2, 0.25)
        self.motion.setAngles("LElbowYaw", -1.7, 0.25)
        self.motion.setAngles("LWristYaw", -1.5, 0.25)

        time.sleep(0.8)

        # emphasize the left hand
        self.motion.setAngles("HeadYaw", 0.2, 0.2)
        self.motion.setAngles("LElbowRoll", -0.2, 0.2)
        self.motion.openHand("LHand")

        time.sleep(2)

        self.bring_to_sit(0.5)

    def point_question(self):
        # move left hand to point to tablet
        time.sleep(3.5)
        self.motion.setAngles("LShoulderRoll", 0.3, 0.25)
        self.motion.setAngles("LShoulderPitch", 0.4, 0.25)
        self.motion.setAngles("LElbowRoll", -0.2, 0.25)
        self.motion.setAngles("LElbowYaw", -1.7, 0.25)
        self.motion.setAngles("LWristYaw", -1.5, 0.25)

        self.motion.openHand("LHand")

        # look at the tablet/left hand?
        # self.motion.setAngles("HeadYaw", 0.2, 0.2)
        # Bring back to sit
        time.sleep(2)
        self.prepare_sit_left(0.15)
        time.sleep(2)

        time.sleep(1.5)
        self.bring_to_sit(0.5)

    def conversion(self):
        # move left hand to represent the denominator
        self.motion.setAngles("LShoulderPitch", 0.7, 0.15)
        self.motion.setAngles("LShoulderRoll", 0.15, 0.1)
        self.motion.openHand("LHand")

        time.sleep(0.2)

        # move right hand to represent the whole number
        self.motion.setAngles("RShoulderRoll", -0.35, 0.2)
        self.motion.setAngles("RShoulderPitch", 0.4, 0.2)
        # bring elbow in
        self.motion.setAngles("RElbowRoll", 1.2, 0.2)
        self.motion.openHand("RHand")

        time.sleep(0.2)

        # move left hand to represent the numerator
        self.motion.setAngles("LShoulderPitch", -0.05, 0.2)

        time.sleep(3)
        self.bring_to_sit(0.5)

    def congratulations(self):
        # this bit of code makes the robot thrust its right hand into the air
        self.bring_to_sit(1.0)
        self.motion.closeHand("LHand")
        self.motion.setAngles("RShoulderPitch", -1.0, 0.3)
        self.motion.setAngles("RShoulderRoll", -1.3, 0.3)
        self.motion.setAngles("RElbowRoll", 1.5, 0.3)
        self.motion.setAngles("RWristYaw", 0, 0.3)
        self.motion.setAngles("LShoulderPitch", -1.0, 0.3)
        self.motion.setAngles("LShoulderRoll", 1.3, 0.3)
        self.motion.setAngles("LElbowRoll", -1.5, 0.3)
        self.motion.setAngles("LWristYaw", 0, 0.3)
        self.motion.closeHand("RHand")

        time.sleep(0.1)
        self.motion.setAngles("RShoulderRoll", -1.0, 0.25)
        self.motion.setAngles("RElbowRoll", 1.0, 0.5)
        self.motion.setAngles("LShoulderRoll", 1.0, 0.25)
        self.motion.setAngles("LElbowRoll", -1.0, 0.5)
        """
        time.sleep(0.4)

        self.motion.setAngles("RShoulderRoll", -1.3, 0.3)
        self.motion.setAngles("RElbowRoll", 1.5, 0.3)
        self.motion.setAngles("LShoulderRoll", 1.3, 0.3)
        self.motion.setAngles("LElbowRoll", -1.5, 0.3)

        time.sleep(0.2)
        self.motion.setAngles("RShoulderRoll", -1.0, 0.25)
        self.motion.setAngles("RElbowRoll", 1.0, 0.5)
        self.motion.setAngles("LShoulderRoll", 1.0, 0.25)
        self.motion.setAngles("LElbowRoll", -1.0, 0.5)

        time.sleep(0.4)

        self.motion.setAngles("RShoulderRoll", -1.3, 0.3)
        self.motion.setAngles("RElbowRoll", 1.5, 0.3)
        self.motion.setAngles("LShoulderRoll", 1.3, 0.3)
        self.motion.setAngles("LElbowRoll", -1.5, 0.3)

        time.sleep(0.2)
        self.motion.setAngles("RShoulderRoll", -1.0, 0.25)
        self.motion.setAngles("RElbowRoll", 1.0, 0.5)
        self.motion.setAngles("LShoulderRoll", 1.0, 0.25)
        self.motion.setAngles("LElbowRoll", -1.0, 0.5)

        # self.genSpeech("yay you did it!")
        # time.sleep(2)
        # self.genSpeech("now i will put my hand back down")
        """
        # bring arms down to avoid drastic sit
        self.motion.setAngles("RShoulderPitch", 0.5, 0.2)
        self.motion.setAngles("LShoulderPitch", 0.5, 0.2)
        self.motion.setAngles("RShoulderRoll", 0, 0.2)
        self.motion.setAngles("LShoulderRoll", 0, 0.2)
        time.sleep(3)
        self.bring_to_sit(1.0)

    def hands(self):
        # this bit of code makes the robot put its hands together and head down
        self.bring_to_sit(0.5)
        self.motion.setStiffnesses("Head", 1.0)
        self.genSpeech("i am now sitting")
        time.sleep(0.5)
        self.genSpeech("now i will put my hands together and tilt down my head")
        time.sleep(5)

        # hands together and head down
        # self.motion.openHand("LHand")
        # self.motion.openHand("RHand")
        # self.motion.setAngles("RShoulderPitch", 1.0, 0.5)
        self.motion.setAngles("LShoulderRoll", -0.05, 0.25)
        self.motion.setAngles("RShoulderRoll", 0.05, 0.25)
        self.motion.setAngles("HeadPitch", 0.25, 0.15)
        time.sleep(3.5)
        self.genSpeech("hmmm. maybe try asking for a hint")

        # move back to the original position
        time.sleep(8)
        self.bring_to_sit(0.5)

    def assess(self, what): # should we take the time.sleep() out since we are waiting using post?
        if(what is "correct"):
            randnr = random.randint(0, len(self.right) - 1)
            speech = self.right[randnr]
            # time.sleep(3)
        elif(what is "wrong"):
            randnr = random.randint(0,len(self.wrong_postfix) - 1)
            speech = self.wrong_postfix[randnr]
            # time.sleep(3)
        elif(what is "trouble"):
            randnr = random.randint(0,len(self.trouble)-1)
            speech = self.trouble[randnr]
            # time.sleep(3)
        elif(what is "hint"):
            randnr = random.randint(0,len(self.hint)-1)
            speech = self.hint[randnr]
            # time.sleep(3)
        elif(what is "confused"):
            randnr = random.randint(0,len(self.confused)-1)
            speech = self.confused[randnr]
            # time.sleep(3)
        elif(what is "auto_hint"):
            randnr = random.randint(0,len(self.auto_hint)-1)
            speech = self.auto_hint[randnr]
            # time.sleep(3)
        id = self.genSpeech(speech)
        return [id,speech]

    # this function is now used to determine the intro movement
    def assessQuestion(self, what):
        # print what
        if(what == "Scaling Up"):
            # print "scaling up question, movement"
            self.scale_up()
            # time.sleep(3)
        elif(what == "Scaling Down"):
            self.scale_down_left()
            # time.sleep(3)
        elif(what == "Common Denominator"):
            self.point_question()
            # time.sleep(3)
        elif(what == "Conversion"):
            # UNSURE HERE
            self.point_question()
            # time.sleep(3)
        elif(what == "Adding Like Denominators Word Problem"):
            self.two_fractions()
            # time.sleep(3)
        elif(what == "Subtracting Like Denominators Word Problem"):
            self.two_fractions()
        elif(what == "Adding Like Denominators"):
            self.two_fractions()
            # time.sleep(3)
        elif(what == "Subtracting Like Denominators"):
            self.two_fractions()
        elif(what == "Adding Unlike Denominators Word Problem"):
            self.two_fractions()
            # time.sleep(3)
        elif(what == "Subtracting Unlike Denominators Word Problem"):
            self.two_fractions()
        elif(what == "Adding Unlike Denominators"):
            self.two_fractions()
            # time.sleep(3)
        elif(what == "Subtracting Unlike Denominators"):
            self.two_fractions()
            # time.sleep(3)

    def introQuestion(self, what):
        randnr = random.randint(0,len(self.question_intro)-1)
        speech = self.question_intro[randnr]
        speech = speech + what + ". Here it is!"
        id = self.genSpeech(speech)
        return [id,speech]

    def introQuestionGeneric(self):
        randnr = random.randint(0,len(self.question_intro_generic)-1)
        speech = self.question_intro_generic[randnr]
        id = self.genSpeech(speech)
        return [id,speech]

    def assessHint2(self, what):
        if (what == "Scaling Up"):
            self.numerator_denominator()
            time.sleep(1)
        elif (what == "Scaling Down"):
            self.numerator_denominator()
            time.sleep(1)

    def assessHint3(self, what):
        if(what == "Conversion"):
            self.conversion()
            time.sleep(1)
        elif (what == "Common Denominator"):
            time.sleep(4)
            self.multiples()
            time.sleep(1)

    def ask(self, question):
        self.genSpeech(question)
        time.sleep(2)


    def releaseNao(self):
        try:
            self.posture.goToPosture("Sit", 1.0)
            self.motion.stiffnessInterpolation("Body",0.0,self.stiffness)
        except Exception, e:
            print "Error when sitting down nao and making nao unstiff: "+str(e)

    def stretchBreak(self):
        self.bring_to_sit(0.2)
        # I intentionally mispelled "lead" to make the speech clearer!
        self.genSpeech("Follow my leed!")
        time.sleep(2.5)
        self.genSpeech("First spread your left arm out.")
        self.motion.setAngles("RShoulderPitch", -1.0, 0.1)
        self.motion.setAngles("RShoulderRoll", -1.2, 0.1)
        self.motion.setAngles("RElbowRoll", 0.0, 0.1)
        time.sleep(4.0)
        self.genSpeech("Now spread your right arm out.")
        self.motion.setAngles("LShoulderPitch", -1.0, 0.1)
        self.motion.setAngles("LShoulderRoll", 1.2, 0.1)
        self.motion.setAngles("LElbowRoll", 0.0, 0.1)
        time.sleep(3.0)
        self.genSpeech("Hold this position for a few seconds.")
        time.sleep(7.0)

        self.genSpeech("Raise both arms up.")
        self.motion.setAngles("RShoulderRoll", 0.0, 0.1)
        self.motion.setAngles("LShoulderRoll", 0.0, 0.1)
        time.sleep(3.0)
        self.genSpeech("Stretch your fingers out too!")
        self.motion.openHand("RHand")
        self.motion.openHand("LHand")
        self.genSpeech("Count to 10 with me while we keep our arms up.")
        time.sleep(4.0)
        for i in xrange(10):
            self.genSpeech(str(i + 1))
            time.sleep(1)
        self.motion.closeHand("RHand")
        self.motion.closeHand("LHand")

        self.genSpeech("Drop your arms down one at a time.")
        time.sleep(3.0)
        self.genSpeech("Start with your left arm.")
        self.motion.setAngles("RShoulderPitch", 0.2, 0.1)
        time.sleep(4.0)
        self.genSpeech("Then your right arm.")
        self.motion.setAngles("LShoulderPitch", 0.2, 0.1)
        time.sleep(4.0)

        self.genSpeech("Rotate your arms so that your palms are facing each other.")
        time.sleep(3.0)
        self.motion.setAngles("RElbowYaw", 1.5, 0.1)
        self.motion.setAngles("LElbowYaw", -1.5, 0.1)
        time.sleep(4.0)

        self.genSpeech("Bend your left elbow.")
        self.motion.setAngles("RElbowRoll", 1.4, 0.1)
        time.sleep(4.0)
        self.genSpeech("Then pull your left arm back.")
        self.motion.setAngles("RShoulderRoll", -1.2, 0.1)
        time.sleep(5.0)
        self.genSpeech("Now bend your right elbow.")
        self.motion.setAngles("LElbowRoll", -1.4, 0.1)
        time.sleep(4.0)
        self.genSpeech("Pull your right arm back just like your left.")
        self.motion.setAngles("LShoulderRoll", 1.2, 0.1)
        time.sleep(5.0)
        self.genSpeech("Straighten your arms out.")
        self.motion.setAngles("LElbowRoll", 0.0, 0.1)
        self.motion.setAngles("RElbowRoll", 0.0, 0.1)
        time.sleep(5.0)
        self.genSpeech("Now drop your arms to your side.")
        self.motion.setAngles("LShoulderPitch", 1.4, 0.1)
        self.motion.setAngles("LShoulderRoll", 0.5, 0.1)
        self.motion.setAngles("RShoulderPitch", 1.4, 0.1)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.1)
        time.sleep(5.0)
        self.genSpeech("Turn your head to the left.")
        self.motion.setAngles("HeadYaw", -0.7, 0.1)
        time.sleep(4.0)
        self.genSpeech("Now to the right.")
        self.motion.setAngles("HeadYaw", 0.7, 0.1)
        time.sleep(4.0)
        self.genSpeech("Bring it back to the center and look up.")
        self.motion.setAngles("HeadYaw", 0.0, 0.1)
        self.motion.setAngles("HeadPitch", -0.5, 0.1)
        time.sleep(4.0)
        self.genSpeech("Let's count to 10 one more time.")
        time.sleep(4.0)
        for i in xrange(10):
            self.genSpeech(str(i + 1))
            time.sleep(1)
        id = self.genSpeech(
            "Great job following along! I hope that was relaxing. Let's get back to our math "
            "problems now. Click the button at the bottom of the tablet to return to the tutoring "
            "session."
        )

        # get back to sitting position to avoid side jaunt
        time.sleep(2)
        self.prepare_sit_right(0.15)
        self.prepare_sit_left(0.15)
        time.sleep(2)

        self.speechDevice.wait(id, 0)

    def breathe_in_guide(self):
        # start position of the arm
        # self.motion.setAngles("HeadPitch", 0, 0.15)
        self.motion.setAngles("RShoulderPitch", 0.4, 0.1)
        self.motion.setAngles("RShoulderRoll", -0.5, 0.1)
        self.motion.setAngles("RElbowYaw", 1.5, 0.1)
        self.motion.setAngles("RElbowRoll", 0.3, 0.1)
        self.motion.setAngles("RWristYaw", 1.6, 0.1)
        self.motion.openHand("RHand")

        # time.sleep(0.3)

        # end position of the arm
        self.motion.setAngles("RShoulderPitch", 0.4, 0.1)
        self.motion.setAngles("RElbowRoll", 1.3, 0.1)

        # time.sleep(2)

        # self.bring_to_sit(0.5)

    def breathe_out_guide(self):
        # start position of the arm
        # self.motion.setAngles("HeadPitch", 0, 0.15)

        # time.sleep(2.5)

        self.motion.setAngles("RShoulderPitch", 0.4, 0.1)
        # self.motion.setAngles("RShoulderRoll", -0.5, 0.1)
        # self.motion.setAngles("RElbowYaw", 1.5, 0.1)
        # self.motion.setAngles("RElbowRoll", 0.4, 0.1)
        self.motion.setAngles("RWristYaw", -1.0, 0.1)
        self.motion.openHand("RHand")

        # time.sleep(0.3)

        self.motion.setAngles("RShoulderPitch", 1.2, 0.1)
        # self.motion.setAngles("RShoulderRoll", -0.5, 0.2)
        # self.motion.setAngles("RElbowYaw", 1.5, 0.2)
        # self.motion.setAngles("RElbowRoll", 0.4, 0.2)
        self.motion.setAngles("RWristYaw", -1.0, 0.1)
        self.motion.openHand("RHand")

        # time.sleep(0.5)

        # raise up elbow to avoid collision
        # self.motion.setAngles("RElbowRoll", 0.8, 0.2)
        # self.motion.setAngles("RWristYaw", 1.3, 0.2)

        # self.bring_to_sit(0.5)

    def mindfulnessBreak(self):
        self.bring_to_sit(0.2)
        self.look()
        # I intentionally mispelled "lead" to make the speech clearer!
        self.genSpeech("Follow my leed!")
        time.sleep(1.5)

        # TODO: fill in rest of break here with speech and motions
        self.genSpeech("Lets start by sitting up nice and straight.")
        id = self.genSpeech("You can rest your hands on your lap.")
        self.speechDevice.wait(id, 0)

        self.genSpeech("We are going to take five deep breaths nice and slow.")
        self.genSpeech("I will guide you and say the word.")
        self.genSpeech("In.")
        id = self.genSpeech(" when you breathe in. and i will say the word. out. when you breathe out.")
        self.speechDevice.wait(id, 0)

        id = self.genSpeech("Lets start. breathe.")
        id = self.genSpeech(" IN.")
        self.breathe_in_guide()
        self.speechDevice.wait(id, 0)
        time.sleep(4.0)
        id = self.genSpeech(" and. ")
        id = self.genSpeech(" out.")
        self.breathe_out_guide()
        self.speechDevice.wait(id, 0)
        # three breaths
        for i in range(3):
            time.sleep(3.0)
            self.breathe_in_guide()
            id = self.genSpeech("breathe. in. ")
            self.speechDevice.wait(id, 0)
            time.sleep(4.0)
            id = self.genSpeech(" and. out.")
            self.breathe_out_guide()
            self.speechDevice.wait(id, 0)

        # fifth (last) breath
        time.sleep(3.0)
        self.breathe_in_guide()
        id = self.genSpeech("One more breath. breathe. in. ")
        self.speechDevice.wait(id, 0)
        time.sleep(4.0)
        id = self.genSpeech(" and. out.")
        self.breathe_out_guide()
        self.speechDevice.wait(id, 0)

        # raise up elbow to avoid collision (will remove after testing)
        # self.motion.setAngles("RShoulderPitch", 0.8, 0.6)
        # self.motion.setAngles("RElbowRoll", 0.8, 0.6)
        # self.motion.setAngles("RWristYaw", 1.3, 0.6)
        # self.posture.goToPosture("Sit", 0.6)

        # get back to sitting position to avoid side jaunt
        time.sleep(2)
        self.prepare_sit_right(0.15)
        self.prepare_sit_left(0.15)
        time.sleep(2)

        self.look()

        self.genSpeech("Now as we stay relaxed, notice the sounds you can hear and how you are feeling.")
        self.genSpeech("Focus on what is happening right now. ")

        self.genSpeech("Now slowly sit back in your chair and relax your body. ")

        id = self.genSpeech("You did a great job following along with that relaxation exercise. Why don't we get back "
                            "to our math activity now. Go ahead and click the button at the bottom of the screen to get back "
                            "to the session."
                            )
        self.speechDevice.wait(id, 0)

    # call before bringing to sit to avoid scooching sit, if the only motions away from sit were using arm joints
    # essentially the same as self.posture.goToPosture("Sit", 0.15), except this won't move the legs
    # uses the angle settings from ALMotionProxy::getSummary() when the robot is in a sitting position, rounded to 2nd decimal
    def prepare_sit_right(self, speed):
        self.motion.setAngles("RShoulderRoll", -0.27, speed)
        self.motion.setAngles("RShoulderPitch", 0.919, speed)
        self.motion.setAngles("RElbowRoll", 1.25, speed)
        self.motion.setAngles("RElbowYaw", 0.5, speed)
        self.motion.setAngles("RWristYaw", 0.01, speed)

    def prepare_sit_left(self, speed):
        self.motion.setAngles("LShoulderRoll", 0.26, speed)
        self.motion.setAngles("LShoulderPitch", 0.885, speed)
        self.motion.setAngles("LElbowRoll", -1.2, speed)
        self.motion.setAngles("LElbowYaw", -.466, speed)
        self.motion.setAngles("LWristYaw", -0.006, speed)

    def bring_to_sit(self, speed=0.15):
        self.prepare_sit_right(speed)
        self.prepare_sit_left(speed)

    def thinkingNod(self):
        #self.motion.wakeUp()
        names = ["LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "HeadPitch", "HeadYaw"]
        angleLists = [[-0.3142], [0.7], [-1.5446], [-0.5], [0.5], [0.0, 0.15, 0.0, 0.15, 0.0], [-0.25]]
        times = [[1.0], [1.0], [1.0], [1.0], [1.0], [1.5, 2.0, 2.5, 3.0, 3.5], [1.0]]
        isAbsolute = True
        self.motion.closeHand("LHand")
        self.motion.angleInterpolation(names, angleLists, times, isAbsolute)
        self.prepare_sit_left(0.5)
        time.sleep(1)
        #self.motion.rest()

    def smallNod(self):
        #self.motion.wakeUp()
        names = ["HeadPitch"]
        angleLists = [[0.37, 0.5, 0.37, 0.5, 0.37]]
        times = [[1.5, 2.0, 2.5, 3.0, 3.5]]
        isAbsolute = True
        self.motion.angleInterpolation(names, angleLists, times, isAbsolute)
        self.prepare_sit_left(0.5)
        time.sleep(1)

    def smallFastNod(self):
        #self.motion.wakeUp()
        names = ["HeadPitch"]
        angleLists = [[0.37, 0.5, 0.37, 0.5, 0.37, 0.5, 0.37, 0.5, 0.37]]
        times = [[1.0, 1.25, 1.5, 1.75, 2.0, 2.3, 2.7, 3.0, 3.25]]
        isAbsolute = True
        self.motion.angleInterpolation(names, angleLists, times, isAbsolute)
        self.prepare_sit_left(0.5)
        time.sleep(1)

    def lookNod(self):
        #self.motion.wakeUp()
        self.motion.setAngles("HeadYaw", 0.0, 0.1)
        self.motion.setAngles("HeadPitch", 0.5, 0.1)
        time.sleep(2)
        self.nod()
        #self.motion.rest()

    def nodSlow(self):
        #self.motion.wakeUp()
        names = ["HeadPitch", "HeadYaw"]
        angleLists = [[0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0], [-0.25]]
        times = [[0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5], [1.0]]
        isAbsolute = True
        self.motion.angleInterpolation(names, angleLists, times, isAbsolute)
        #self.motion.rest()

    def rightExplain(self):
        # moving the right hand to the beginning position
        self.motion.setAngles("HeadPitch", 0, 0.15)
        self.motion.setAngles("RShoulderRoll", -0.2, 0.1)
        self.motion.setAngles("RShoulderPitch", 0.3, 0.1)
        self.motion.setAngles("RElbowRoll", .8, 0.1)
        self.motion.setAngles("RElbowYaw", 1.2, 0.1)
        self.motion.setAngles("RWristYaw", 1.2, 0.1)

        time.sleep(0.6)

        # emphasize the right hand
        self.motion.setAngles("HeadYaw", -0.6, 0.2)
        self.motion.setAngles("RElbowRoll", 0.2, 0.2)

        time.sleep(0.5)

        self.motion.openHand("RHand")

    def leftExplain(self):

        # moving the left hand to the beginning position
        self.motion.setAngles("LShoulderRoll", 0.2, 0.1)
        self.motion.setAngles("LShoulderPitch", 0.3, 0.1)
        self.motion.setAngles("LElbowRoll", -0.8, 0.1)
        self.motion.setAngles("LElbowYaw", -1.2, 0.1)
        self.motion.setAngles("LWristYaw", -1.2, 0.1)

        time.sleep(0.6)

        # emphasize the left hand
        self.motion.setAngles("HeadYaw", 0.6, 0.2)
        self.motion.setAngles("LElbowRoll", -0.2, 0.2)

        # open both hands and move the head back center
        
        self.motion.openHand("LHand")

        time.sleep(0.6)

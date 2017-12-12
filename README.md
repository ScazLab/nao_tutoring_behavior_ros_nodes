# nao_tutoring_behavior_ros_nodes

This repository contains code for the ROS nodes controlling the tutoring behavior study. 
It is set up to run in Docker so that it may be run on any operating system, though the ROS code could be pulled out of the Docker container if being run on a Linux machiene.

# Running in Docker

To run the docker container, run the command:
```
docker run -it -p 9090:9090 -v  /*path_to*/nao_tutoring_behavior_ros/catkin_ws:/root/catkin_ws nao_tutoring_behavior
```

-p 9090:9090 specifies that port 9090 on your local machiene is mapped to port 9090 in the container. This is important to note for running the tablet server later.

In a second terminal, run:
```
docker ps -l
```

This will show you a list of docker containers open. Find the name of the open container in the last column. 
Then run:
```
docker exec -it *name_of_container* bash
```

Open two more docker terminals. Before running the ros nodes, you must first run:
```
source ./devel/setup.bash
```

In your four docker terminals, run the following commands (one per terminal):
```
roscore
rosrun nao_tutoring_behaviors node_model_dummy.py 
rosrun nao_tutoring_behaviors node_tablet.py 
rosrun nao_tutoring_behaviors node_robot_controller.py [--robot]            (the --robot flag indicates that the node should connect to Nao)
```

The ROS nodes are now running. In the terminal running node_tablet.py, you should see that it is waiting for the tablet to connect.

# Messages

## ROS message types
The nodes communicate via the following message streams:

robot_speech_msg : the robot publishes here to indicate when it begins and finishes speaking. This allows the tablet to disable buttons during this time.

robot_lesson_msg : the robot publishes here when it speaks to give an example. This lets the tablet react to robot speech during examples and tutorials to show the right steps at the next time. The model node does not subscribe to this.

robot_inactivity_msg : the robot publishes here when it speaks during other activities, like during a tictactoe game. This is separate so that the model is not spammed with too many messages between tablet node and robot during an activity. The model node does not subscribe to this.

These three prior messages are all simple strings.

tablet_msg : the tablet node publishes here to inform the model and the robot of what the tablet is doing and what actions should be taken. It sends a TabletMsg which contains a msgType (string), questionNumOrPart (int), robotSpeech (string) and a string of otherInfo.
Examples of msgTypes are 'CA' indicating a correct answer, 'IA' indicating an incorrect answer, 'TICTATOE-[WIN/LOSS]' indicating the end of a tictactoe game, 'START' to indicate the start of the session, or 'SHOWING-QUESTION' to indicate that the tablet has just displayed a question. Both the robot controller and model subscribe to this.

tablet_lesson_msg : the tablet node publishes here during an example to instruct the robot what to say. This is separate so that the robot can respond with a the robot_lesson_msg to trigger the next step, and so as to avoid sending too many unneeded messages to the model. 

tablet_inactivity_msg: the tablet node publishes here during other activities, like tic tac toe. Only the robot subscribes.

model_decision_msg : the model publishes the next action that should be taken. This is in the form of a ControlMsg, which contains a nextStep (string), questionNum, questionLevel, a string of robotSpeech and string of otherInfo.

Currently the "otherInfo" in both TabletMsgs and ControlMsgs is unused so I will probably remove it in a next commit.

## Model to node_tablet messages:

The model sends messages via model_decision_msg. These come in the form of ControlMsgs, which have a string nextStep field.

The following nextStep strings are supported to trigger tutoring behaviors:

`QUESTION` This triggers the tablet to move on to the next question. The question number and level are specified via the questionNum and questionLevel fields in ControlMsg, both of which are integers. 

`SHOWSTRUCTURE` This triggers the tablet to show the box structure of the current question. This level of specificity in hint type may not be entirely desirable in the model, but is available if needed. For more general hint giving, see the `SHOWHINT` instruction.

`THINKALOUD` This triggers the robot to ask the student to think about the first step of the problem out loud. The tablet does nothing at this time.

`SHOWHINT` This is a more general hint behavoir. In level one, where box structures are not so helpful, a general text hint is displayed (though this could be substituted by showing the easy tutorial display or printing the associated multiplication fact). In other levels, the box structure is shown, unless it has already been displayed by a previous hint. In that case, some of the boxes are filled in as a hint.

`SHOWEXAMPLE` This prompts a worked example for a different question of a similar level. In level one, the robot states the multiplication fact leading to the division fact. In other levels, it displays the box structure and gradually fills it in while talking through it. A bank of example questions is used for each level, but because the box structure and intermediate steps are generated automatically ( in `example_generation.py`), all that must be specified there is a numerator and denominator. These examples are non interactive, so the robot does the entire problem for the student before telling it to return to the question on which they were working.

`SHOWTUTORIAL` This starts an interactive tutorial to be displayed. In level one, this shows a set of boxes filled with balls which illustrates the division problem. The facts denominator x quotient = numerator and numerator / denominator = quotient are display with the quotient missing for the student to fill in. In other levels, the box structure is displayed with only some boxes enabled. The student is told to fill in the enabled boxes and is allowed to proceed if all the boxes in the current step are correct. This enables later boxes. In both types of tutorial, the answer is filled in for the student if they get a step wrong 3 times.

`TICTACTOE` This triggers a tic tac toe break. 

## node_tablet to tablet app messages

node_tablet sends messages to the app via TCP messages, which just have the form of a string. Different parts of each instruction are always separated by a semicolon. The following types and formats of instructions are supported.

`"QUESTION;(level);(number)` This message triggers the tablet to display the next question, specified by the given level and question number. The tablet has access to the json containing questions, so this is all of the information that is required to show the question.

`SHOWSTRUCTURE` messages cause the tablet to display the box structure for a problem, but come in different flavors based on the other parts of the instruction
    `SHOWSTRUCTURE;` Given no other information, the tablet will display the structure of the current question
    `SHOWSTRUCTURE;numerator-denominator` will show the box structure for the problem of numerator/denominator. 
    `SHOWSTRUCTURE-TUTORIAL;numerator-denominator;[All-Answers]` will show the box structure for the problem of numerator/denominator. It will only enable the first step of boxes to be filled in because they are being used in a tutorial. 
    The All-Answers string contains information about the correct input to each box so that student iput can be verified, and is formatted as follows. Each box is specified by "line_number-box_numer-answer" and these parts are separated by colons. Thus, to specify that box(1,1) = 1, box (1,2) = 2 and box(2,1) = 3, this string would be "1-1-1-:1-2-2:2-1-3". These strings are generated in `example_generation.py`.
    
`FILLSTRUCTURE` this instruction fills in boxes in the box structure but also comes in different variantions 
    `FILLSTRUCTURE;` with no other information provided, this will fill in the answers to all of the boxes that are enabled. This is used to fill in the next step in a tutorial if a student has gotten the answer incorrect enough times.
    `FILLSTRUCTURE;Steps to fill in;` will fill in the indicated boxes with the provided answer. This string is formatted the same was as All-Answers above, except that it only specifies the boxes that are relevant to the current step. This message is used in worked examples.
    `FILLSTRUCTURE;EASY;part-answer;` will fill in the box of the easy tutorial corresponding to the given part number.

`SHOWTEXTHINT;(insert text here);` This will cause the tablet to display the string after the semi colon as plain text in the hint pane. It is used to display some of the steps of easy examples, which do not have any other visuals to accompany them, and can be used in level one hints, which are more simple.

`SHOWEASYTUTORIAL;numerator-denominator` will cause the tablet to display the easy tutorial visuals, which include denominator many boxes with numerator many balls split between them. 

## tablet app to node_tablet messages

The tablet sends messages to the node_tablet server as TCP string messages. 
The following types of messages are handled:

`START` This indicates that the session has started. This message can later also contain data about the session that is provided on the start screen, after the semi colons, depending on what information needs to be recorded at the start of a session. node_tablet will also pass this information on to the model and robot, causing the Nao to introduce himself.

`SHOWING-QUESTION` is sent when a question is shown (either because the session was just started or because the student hit the "Next Question" button. This allows the robot to wait to read the question until it is on the screen.

`CA` indicates the student entered a correct answer

`IA` indicates the student entered an incorrect answer

`TICTACTOE` these messages control the tictactoe game. This message flow is the same as in previous versions of the app. All of the computation is handled by the tablet and the server only passes this message along to the robot so it can speak.
  `TICTACTOE-WIN` and `TICTACTOE-LOSS` indicate the end of a game.
  
`TUTORIAL-STEP;result` messages indicate progress on an interactive tutorial. `result` can be the string "CORRECT", "INCORRECT" or "INCOMPLETE". If the first part of the message is actually "TUTORIAL-STEP-EASY" it refers to an easy (level 1) tutorial.

# Most important files and locations:
There are a lot of directories generated by ROS, so here are the paths of the most relevant and important things:
message types are all in /catkin_ws/src/nao_tutoring_behaviors/msg

The three node files are in:

/catkin_ws/src/nao_tutoring_behaviors/scripts/node_robot_controller.py
/catkin_ws/src/nao_tutoring_behaviors/scripts/node_tablet.py
/catkin_ws/src/nao_tutoring_behaviors/scripts/node_model_dummy.py

Code to generate the steps of division examples (e.g. finding the values that should go in structure boxes) is in:
/catkin_ws/src/nao_tutoring_behaviors/scripts/example_generation.py

# Running Nao
These instructions are adopted from [this previous project's README](https://github.com/ScazLab/nao_tutoring#nao-tutoring) which was written by Arsalan Sufi. They have been edited for relevance to the current project.

## Installation Instructions: Python Scripts

The project's Python scripts are located in the `python_scripts/` directory.

After cloning the repository, within the folder python_scripts/, type 'mkdir data'.
This creates a data directory that nao_server.py will look for when running the code.

### Installing naoqi

To run Nao code, the only module that you should need to install is `naoqi` (the Python NAOqi SDK).
This module can't be installed using `pip`.
You'll need to download the module from the [Aldebaran website](https://www.aldebaran.com/en).
Aldebaran provides installation instructions [here](http://doc.aldebaran.com/1-14/dev/python/install_guide.html).

Below are a few notes that supplement the linked instructions:

* To access the zip archives mentioned in Step 2, you'll need to log in to the [Aldebaran website](https://www.aldebaran.com/en).
  Ask someone in the lab for the login credentials.

* Once you've logged in, navigate to *Resources > Software > Python NAOqi SDK*.
  Download a zip archive for **version 1.14.5** of the SDK.
  This is the version used by the project.

* Step 3 makes sure that Python can find the `naoqi` module.
  After you've downloaded and unzipped your zip archive, the following command should do the trick for both Linux and MacOS:
    ```
    export PYTHONPATH=${PYTHONPATH}:/path/to/sdk/
    ```
  where `/path/to/sdk/` is the path to your unzipped zip archive.
  You can add this line to your `.bash_profile` or `.bashrc` to avoid having to type it into every new terminal.

### Running nao_server.py

You should now be able to run the ROS nodes as described above. The IP address is found in the code by `socket.gethostname()`, and the port is hard coded to 9090.
To determine your IP address, so that you can connect a tablet app connection to the socket, you can use `ifconfig` in your terminal.

**A quick but important note about Python binaries.**
The Python binary that you use to run the script should be a system binary.
A brew-installed binary likely won't work.
When I use my brew-installed Python 2.7 to run the script on Yosemite, Python crashes.

To check which Python binary you're actually using, type `which python` into your terminal.
If the binary is located in `/usr/local/bin/`, it's probably brew-installed.
To get the terminal to use the system binary instead, run
```
/usr/bin/python ...
```
instead of
```
python  ...
```

# Dockerfile for running the Computational Tutoring Project
# Aditi Ramachandran
# Last Updated: 2/16/18
#
# Usage: 
#   docker build -t nao_tutoring_behavior:latest .
#   docker run -it -p 9090:9090 -v /Users/aditi/Documents/phd_research/comp-tutoring-project/ros_nodes/catkin_ws:/home/ros/catkin_ws nao_tutoring_behavior:latest
#
# To open another terminal from the same container:
#   docker ps
#   docker exec -it [container_name] bash
#
# ROS message sending: 
#     rostopic pub /building_trust/game_command building_trust_nao_controller/GameCommand "{player_number: 1, command: 0, properties: '{\"strategy\": \"success\", \"agent-type\": \"human\"}'}"
#     rostopic pub /building_trust/game_command building_trust_nao_controller/GameCommand "{player_number: 1, command: 1, properties: '{\"result\": \"failure\", \"num-success-rounds\": 0, \"num-total-rounds\": 1}'}"
#     rostopic pub /building_trust/game_event building_trust_nao_controller/GameEvent "{player_number: 1, gameEvent: 1, properties: '{\"result\": \"success\"}'}"
#     rostopic pub /building_trust/robot_behavior building_trust_nao_controller/RobotBehavior "{action: 0}"
#     rostopic pub /building_trust/game_command building_trust_nao_controller/GameCommand "{player_number: 1, command: 3, properties: ''}"
#     rostopic pub /building_trust/game_command building_trust_nao_controller/GameCommand "{player_number: 1, command: 4, properties: '{\"result\": \"success\"}'}"
#     rostopic pub /building_trust/game_command buildg_trust_nao_controller/GameCommand "{player_number: 1, command: 1, properties: '{\"result\": \"success\", \"num-success-rounds\": 1, \"num-total-rounds\": 2}'}"


FROM ros:indigo-robot
FROM ros:indigo-perception

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# Set up work directory
RUN useradd ros
RUN mkdir /home/ros && chown -R ros: /home/ros
ENV HOME "/home/ros"
ENV PATH="/home/ros/.local/bin:${PATH}"
WORKDIR   /home/ros


RUN apt-get update
RUN apt-get -y install build-essential python-pip
RUN pip install pip --upgrade
RUN pip install ipython


#RUN apt-get -y install wget
#RUN apt-get -y install ros-indigo-tf
#RUN apt-get -y install libsndfile1-dev libpng12-dev
#RUN apt-get -y install libyaml-cpp-dev
#RUN apt-get -y install ros-indigo-cv-bridge
#RUN apt-get -y install ros-indigo-image-transport
#RUN apt-get -y install ros-indigo-camera-info-manager
#RUN apt-get -y install ros-indigo-octomap-msgs
#RUN apt-get -y install ros-indigo-nao-robot
#RUN apt-get -y install ros-indigo-nao-extras
#RUN apt-get -y install ros-indigo-rosbridge-server

USER ros
RUN mkdir catkin_ws
RUN mkdir src
RUN cd src && git clone https://github.com/ScazLab/task-models.git

RUN cd src/task-models/ && python setup.py develop --user

RUN cd src/ && git clone https://github.com/ScazLab/pomdp-solve
RUN mkdir src/pomdp-solve/build
RUN cd src/pomdp-solve/build/ && ../configure --prefix=$HOME/.local
RUN cd src/pomdp-solve/build/ && make
RUN cd src/pomdp-solve/build/ && make install

WORKDIR /home/ros/catkin_ws




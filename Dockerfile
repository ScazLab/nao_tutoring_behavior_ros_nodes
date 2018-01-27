# Dockerfile for running the Repairing Trust Experiment
# Sarah Strohkorb
# Last Updated: 10/19/17
#
# Usage: 
#   docker build -t nao_tutoring_behavior .
#   docker run -it -p 9090:9090 -v /Users/aditi/Documents/phd_research/comp-tutoring-project/ros_nodes/nao_tutoring_behavior_ros_nodes/catkin_ws:/root/catkin_ws nao_tutoring_behavior 
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


FROM ros:indigo-ros-base

MAINTAINER Aleksandra Zakrzewska “aleksandra.zakrzewska@yale.edu”

RUN apt-get install sudo
RUN sudo apt-get update
RUN sudo apt-get -y install build-essential
RUN sudo apt-get -y install wget
RUN sudo apt-get -y install ros-indigo-tf
RUN sudo apt-get -y install libsndfile1-dev libpng12-dev
RUN sudo apt-get -y install libyaml-cpp-dev
RUN sudo apt-get -y install ros-indigo-cv-bridge
RUN sudo apt-get -y install ros-indigo-image-transport
RUN sudo apt-get -y install ros-indigo-camera-info-manager
RUN sudo apt-get -y install ros-indigo-octomap-msgs
RUN sudo apt-get -y install ros-indigo-nao-robot
RUN sudo apt-get -y install ros-indigo-nao-extras
RUN sudo apt-get -y install ros-indigo-rosbridge-server


WORKDIR /root/catkin_ws




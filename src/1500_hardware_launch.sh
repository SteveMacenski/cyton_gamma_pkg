#!/bin/bash

source /opt/ros/indigo/setup.bash
source /home/rdtintern/catkin_ws/devel/setup.bash

echo "[1/6] start controller manager"

xterm -e roslaunch cyton_1500_controllers controller_manager.launch &

sleep 10

echo "[2/6] start controllers"

xterm -e  roslaunch cyton_1500_controllers start_controller.launch &

sleep 10

echo "[3/6] start joint state publisher"

xterm -e rosrun cyton_1500_controllers dynamixel_joint_state_publisher.py &

sleep 5

echo "[4/6] start planning execution window"

xterm -e roslaunch cyton_gamma_pkg gamma_1500_planning_execution.launch 2> /dev/null &

sleep 20

echo "[5/6] start command front end"

xterm -e python /home/rdtintern/catkin_ws/src/cyton_gamma_pkg/src/command_front_end.py 2> /dev/null &

echo "[6/6] start feedback front end"

xterm -e python /home/rdtintern/catkin_ws/src/cyton_gamma_pkg/src/feedback_front_end.py 2> /dev/null &

echo "ready!"

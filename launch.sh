#!/bin/bash

source /home/$USER/.bashrc
source /home/$USER/venvs/blinka_venv/bin/activate
source /home/$USER/ros2_jazzy/install/local_setup.bash
source /home/$USER/proto-ros2/install/local_setup.bash

ros2 run screen_dispatcher_pkg screen_dispatcher

#!/bin/bash

source /home/$USER/.bashrc
source /home/$USER/venvs/blinka_venv/bin/activate
source /home/$USER/ros2_jazzy/install/setup.bash
source /home/$USER/proto-ros2/install/setup.bash

ros2 run screen_dispatcher_pkg screen_dispatcher &
echo "Screen Dispatcher Started!" &
ros2 run face_renderer_pkg face_renderer &
echo "Face Renderer Started!" &
ros2 run mic_input_pkg mic_input &
echo "Mic Input Started!" &
ros2 run facial_expression_detector_pkg facial_expression_detector &
echo "Face Detector Started!" &
echo "Audio Detector Started!" &
ros2 run audio_expression_detector_pkg audio_expression_detector
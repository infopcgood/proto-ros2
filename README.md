# proto-ros2
Open source software to control various protogen fursuit heads on top of ROS2 / RPi5.

## Hardware Setup
This software assumes the following hardware setup:
- Raspberry Pi 5
- Adafruit RGB Matrix Bonnet
- 2 64x32 HUB75 LED Matrices daisy-chaine
- A camera module or a webcam
- A speaker (optional)

Consult [this page](https://learn.adafruit.com/rgb-matrix-panels-with-raspberry-pi-5/wiring) for wiring instructions.

## Software Setup
> This paragraph assumes that you have some Linux/UNIX knowledge(preferably Debian or Ubuntu). If you don't, feel free to contact me for help.

First, setup the SD card and the Python libraries for the LED matrices by following [this page](https://learn.adafruit.com/rgb-matrix-panels-with-raspberry-pi-5/raspberry-pi-5-setup). After the installation, run a [basic test](https://learn.adafruit.com/rgb-matrix-panels-with-raspberry-pi-5/basic-test) to check if it's correctly set up. After this step, add the `activate` executable(usually located at `(venv_dir)/bin/activate`) to `.bashrc`.

Next, download and compile ROS2 from the source. It sounds frightening, but it's just a bunch of commands. I followed [this guide](https://forums.raspberrypi.com/viewtopic.php?t=361746) and everything went well. Just change `iron` to whatever distribution you want to use. I used jazzy, so my command should look like this:
```bash
sudo apt install -y git colcon python3-rosdep2 vcstool wget python3-flake8-docstrings python3-pip python3-pytest-cov python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-import-order python3-flake8-quotes python3-pytest-repeat python3-pytest-rerunfailures python3-vcstools libx11-dev libxrandr-dev libasio-dev libtinyxml2-dev

mkdir -p ~/ros2_jazzy/src

cd ~/ros2_jazzy

vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src

sudo rm /etc/ros/rosdep/sources.list.d/20-default.list

sudo apt upgrade

sudo rosdep init

rosdep update

rosdep install --from-paths src --ignore-src --rosdistro jazzy -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"

colcon build --symlink-install
```
While running `colcon build --symlink-install`, some(or many) packages may fail because of missing dependencies. Look at the error message carefully and install the required dependencies from either `apt` or `pip`. Rerunning doesn't delete already compiled packages, so try over and over. This may take a few hours.

After every dependency issue is resolved, run `colcon build --symlink-install` to double-check that all packages are compiled successfully.

Lastly, install opencv, imutils and dlib as we will need it for face landmark recognition. Follow the following commands that I copied and modified from [this guide](https://pyimagesearch.com/2017/05/01/install-dlib-raspberry-pi/). Make sure you are in the venv that you created while installing the LED Matrix library!
```bash
sudo apt-get update
sudo apt-get install build-essential cmake
sudo apt-get install libgtk-3-dev
sudo apt-get install libboost-all-dev

pip install numpy
pip install scipy
pip install scikit-image
pip install dlib
```

## Installation
Clone this repository into a directory of your choice, and compile it using `colcon build`.
```bash
git clone https://github.com/infopcgood/proto-ros2
colcon build
```
After building, add `source (repository_directory)/install/local_setup.bash` to your `.bashrc`. Exit and reopen the shell.

TODO: startup using crontab

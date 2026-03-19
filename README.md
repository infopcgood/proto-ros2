# proto-ros2
Open source software to control various protogen fursuit heads on top of ROS2 / RPi5.

## Hardware Setup
This software assumes the following hardware setup:
- Raspberry Pi 4/CM4/5/CM5
- A camera module or a webcam
- A microphone
- A speaker (optional)
### For HUB75 Display Setups Only
Currently, this repository only supports Raspberry Pi 5/CM5 for HUB75 display setups. Raspberry Pi 4/CM4 support will be added in the future.
- Adafruit RGB Matrix Bonnet
- 2 64x32 HUB75 LED Matrices daisy-chained
Consult [this page](https://learn.adafruit.com/rgb-matrix-panels-with-raspberry-pi-5/wiring) for wiring instructions.
### For `proto-panel` Display Setups Only
- TODO
### Other Setups
- TODO

## Software Setup
> This paragraph assumes that you have some Linux/UNIX knowledge(preferably Debian or Ubuntu). If you don't, feel free to contact me for help.

First, flash the operating system image onto the microSD card/eMMC according to the **Library Compatibility Table** shown below. When the OS installation is finished, you may create a 'systemwide venv' and add it to `~/.bashrc` or use systemwide Python packages.

Next, download and compile ROS2 from source. The following script is a modified version of [this guide](https://forums.raspberrypi.com/viewtopic.php?t=361746). You can change `jazzy` to a ROS2 distribution of your liking, though I could not ensure backwards compatibility for older distributions.
(*Currently the dependencies aren't optimized and is rather heavy. There will be a more 'thinned down' version of this script in the future.*)
```bash
sudo apt install -y git colcon python3-rosdep2 vcstool wget python3-flake8-docstrings python3-pip python3-pytest-cov python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-import-order python3-flake8-quotes python3-pytest-repeat python3-pytest-rerunfailures python3-vcstools libx11-dev libxrandr-dev libasio-dev libtinyxml2-dev python3-lark libbullet-dev

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
If some packages fail to build while running `colcon build --symlink-install`, look at the error message carefully and install the required dependencies from either `apt` or `pip`. Install the packages and rebuild until every package compiles successfully.

Lastly, install opencv, imutils and dlib as we will need it for face landmark recognition. Follow the following commands that I copied and modified from [this guide](https://pyimagesearch.com/2017/05/01/install-dlib-raspberry-pi/). Make sure you are in the venv that you created while installing the LED Matrix library!
```bash
sudo apt-get install build-essential cmake libgtk-3-dev libboost-all-dev

# For systemwide package setups

# For venv setups
pip install numpy scipy scikit-image dlib
```

HUB75 display users will need to install Python libraries for the LED matrices by following [this page](https://learn.adafruit.com/rgb-matrix-panels-with-raspberry-pi-5/raspberry-pi-5-setup). After the installation, run a [basic test](https://learn.adafruit.com/rgb-matrix-panels-with-raspberry-pi-5/basic-test) to check if it's correctly set up. Make sure to modify the instructions according to your Python setup.

## Installation
Clone this repository into `$HOME/proto-ros2`, and compile it using `colcon build`.
```bash
cd ~
git clone https://github.com/infopcgood/proto-ros2
cd proto-ros2
colcon build
```
After building, add `source (repository_directory)/install/local_setup.bash` to your `.bashrc`. Exit and reopen the shell.

Finally, use a method of your choice(`crontab`, `systemd`, etc...) to start `launch.sh` on startup.

If you would like to use OpenSeeFace based lipsync features, you need to install [OpenSeeFace](https://github.com/emilianavt/OpenSeeFace) and [proto-utils](https://github.com/infopcgood/proto-utils). Clone them to your home directory.
```bash
cd ~
git clone https://github.com/emilianavt/OpenSeeFace
git clone https://github.com/infopcgood/proto-utils
```

## Hardware Setup
TODO

## License
All packages under this repository is distributed via the GPL-3.0 license.

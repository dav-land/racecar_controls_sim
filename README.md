# Racecar Controls Simulator

Assumed clean install of Ubuntu 18.04 and Ros Melodic Desktop Full with `catkin_ws/` set up in home direcotry.

## Getting Started

First we need to install some dependancies
```
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server

sudo apt-get install ros*controller*
```
Next we need to clone the github repo into the src folder
```
cd ~/catkin_ws/src
git clone https://github.com/dav-land/racecar_controls_sim.git
cd ..
catkin_make
```
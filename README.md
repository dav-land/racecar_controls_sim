# Racecar Controls Simulator

Assumed clean install of Ubuntu 18.04 and Ros Melodic Desktop Full with `catkin_ws/` set up in home direcotry.

## Getting Started

First we need to install some dependancies
```
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server ros*controller*
```
Next we need to update the server for gazebo located at `~/.ignition/fuel/config.yaml` from https://api.ignitionfuel.org to https://api.ignitionrobotics.org
Next we need to clone the github repo into the src folder
```
cd ~/catkin_ws/src
git clone https://github.com/dav-land/racecar_controls_sim.git
cd ..
catkin_make
```

### Software Used
This was set up using Ubuntu 18.04 using VMware on a Windows 10 machine. In order to get Gazebo to launch in the VM the graphics instruction must be shrunk down using `export SVGA_VGPU10=0`.
To make this change perminate use `echo "export SVGA_VGPU10=0">>~/.profile`
# Racecar Controls Simulator

Assumed clean install of Ubuntu 18.04 and ROS Melodic Desktop Full with `catkin_ws/` set up in home directory.

## Getting Started

First we need to install some dependancies
```
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server ros*controller*
```
Going forward you may or may not run into an error with the gazebo server. If you do you can update the server for gazebo located at `~/.ignition/fuel/config.yaml` from https://api.ignitionfuel.org to https://api.ignitionrobotics.org

Next we need to clone the GitHub repo into the src folder
```
cd ~/catkin_ws/src
git clone https://github.com/dav-land/racecar_controls_sim.git
cd ..
catkin_make
source devel/setup.bash
```

## Running the Simulation

#### Simple PD Controller
To get started with a simple PD controller in the hallway environment run the following command. This will start up the gazebo simulation and necessary controllers. The car should start driving immediately and make it from one end of the hallway to the other.
```
roslaunch controller PDControllerHallway.launch
```
While a PD controller is a great starting point it becomes unreliable the faster the car goes.  In order to see this edit the `controller/src/PDController.py` file, line 52. Change it from `self.cmd_pub.drive.speed = 2.0` to `self.cmd_pub.drive.speed = 6.0`. This will cause the car to not have enough time to register the turn and it will crash into the wall.

### Software Used
This was set up using Ubuntu 18.04 using VMware on a Windows 10 machine. In order to get Gazebo to launch in the VM the graphics instruction must be shrunk down using `export SVGA_VGPU10=0`.
To make this change permanent use `echo "export SVGA_VGPU10=0">>~/.profile`

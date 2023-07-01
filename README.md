# Catchbot

UR5e with robotiq 2 finger gripper on ROS Melodic(melodic branch) and Kinetic(kinetic branch)
1. git clone -b melodic https://github.com/ustcsiwei88/catchbot --recursive
2. cd catchbot
3. rosdep install --from-paths src --ignore-src -r -y
4. catkin\_make
5. source devel/setup.bash

##### Physical robot experiment (C++)
Install ur_rtde
```
$ sudo add-apt-repository ppa:sdurobotics/ur-rtde
$ sudo apt-get update
$ sudo apt install librtde librtde-dev
```

Open Motive and select the rigid body to throw and allow Motive to stream.

In an Ubuntu desktop, install vrpn_ros and set up the server IP address as Motive computer.

Open robot arm and set to remote control.

In the Ubuntu desktop,
```
$ rosrun catchbot catchbot_node 
```

Throw the ball.

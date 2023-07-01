# catchbot

UR5e with robotiq 2 finger gripper on ROS Melodic in simulation
1. cd catchbot
2. git submodule update --init
3. rosdep install --from-paths src --ignore-src -r -y
4. catkin\_make
5. source devel/setup.bash



Open Gazebo simulation

```roslaunch catchbot_gazebo catchbot_gazebo.launch```

Open catchbot_node for catching node

```rosrun catchbot catchbot_node```

Run script for throwing ball

```rosrun catchbot catchbot_setup.py```

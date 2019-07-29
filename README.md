# catchbot

A modification of neka-nat's work(https://github.com/neka-nat/ur\_ws)

UR5e with robotiq 2 finger gripper on ROS Melodic(melodic branch) and Kinetic(kinetic branch)
1. git clone -b melodic https://github.com/ustcsiwei88/catchbot --recursive
2. cd catchbot
3. rosdep install --from-paths src --ignore-src -r -y
4. catkin\_make
5. source devel/setup.bash
6. roslaunch catchbot\_gazebo catchbot\_gazebo.launch
7. rosrun catchbot catchbot\_node
8. rosrun catchbot catchbot\_setup.py

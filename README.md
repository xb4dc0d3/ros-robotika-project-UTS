# UTS Project ROS 

## __Getting Started__
Make sure you have **python2** and **git** installed.
```bash
mkdir catkin_ws && cd catkin_ws
git clone https://github.com/xb4dc0d3/ros-robotika-project-UTS.git
catkin_make
source devel/setup.bash
```

## __Run Application__
### Rviz Plotting
```bash
roslaunch m2wr rviz.launch
```
### Launch Simulation (Gazebo) and Spawn The Robot
```bash
roslaunch gazebo_ros empty_world.launch
roslaunch m2wr spawn.launch
```
### Execute Program
```bash
chmod +x move_uts.py
rosrun m2wr move_uts.py
```


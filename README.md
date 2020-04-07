# UTS Project ROS 

## __Getting Started__
Make sure you have **python2** and **git** installed.
```bash
mkdir catkin_ws && cd catkin_ws
git clone https://github.com/xb4dc0d3/ros-robotika-project-UTS.git
source devel/setup.bash
```

## __Run Application__ PART I

### Launch Simulation (Gazebo) and Spawn The Robot
```bash
roslaunch gazebo_ros empty_world.launch
roslaunch m2wr spawn-no-sensor.launch
```
### Rviz Plotting
```bash
roslaunch m2wr rviz-no-sensor.launch
```

### Execute Program
```bash
rosrun m2wr move_uts.py
```
## __Run Application__ PART II

### Launch Simulation (Gazebo) and Spawn The Robot
```bash
roslaunch m2wr circuit.launch
roslaunch m2wr spawn-no-sensor.launch
```
### Rviz Plotting
```bash
roslaunch m2wr rviz-no-sensor.launch
```

### Execute Program
```bash
rosrun m2wr move_uts.py
```

## __Run Application__ PART II

### Launch Simulation (Gazebo) and Spawn The Robot
```bash
roslaunch m2wr circuit.launch
roslaunch m2wr spawn.launch
```
### Rviz Plotting
```bash
roslaunch m2wr rviz.launch
```

### Execute Program
```bash
rosrun m2wr laser_move.py
```

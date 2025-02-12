# HM2 RL 24/25 Arsen Hudyma

Hi, this is the repository for the HM2 of the course 'Robotics Lab'. For this homework we'll see the iiwa manipulator in the gazebo environment and also we'll controll it in a more advanced way, not by using the ros2_control functionalities but more manually with the KDL library by OROCOS. Specifically, the robot actually can be controlled in terms of position commands or velocities, and the end effector can compute a linear or a circular trajectory in function of a curvilinear abscissa, that can be set by trapezoidal velocity profile or a cubic polinomial one.
However for more details and knowledge on how to do all this things its better if you give a look at my presentation ppt (this too has some useful videos in it), that as always you find in this repository ;)

To start, you'll need this repository on your computer, so get it by:
```bash
git clone https://github.com/arsen-hdm/RL_HM2.git
```

Then, once you're in the dockek container, firstly:
```bash
colcon build
. install/setup.bash
```

### Simulation in gazebo with the position command interface
The position interface is the one used by default, so you simply need to run this command:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true
```
But you want also the robot to move, so the command to do this is:
```bash
ros2 run ros2_kdl_package ros2_kdl_node
```
And in base of the configuration of the control node the robot will move in a certain way.

### Simulation in gazebo with the velocity command interface
For the spawn of the iiwa in a gazebo world the initial command is always the same but we need also to pass some other parameters, the entire command is shown below:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true
```
Also the ros2 node need some parameters to understand that we want to command the robot by velocities, and the command is:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
```

By combining the different type of trajectories and curvilinear abscissas you can obtain 4 different types of movement, as before I redirect you to the presentation so that you can see how I've implemened it and to know how to do it.
Thanks for the attention, you can find also other homeworks in my personal repositories!

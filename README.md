# How to run?

## 1. Build

```bash
<catkin build> or <catkin_make>
```

## 2. Teleoperation side

```bash
source devel/setup.bash
roslaunch dm_controllers load_dm_hw.launch
```

plugin damiao usb2can module

```bash
sudo chmod -R 777 /dev/ttyACM*
roslaunch dm_controllers load_dm_hw.launch
```

## 3. Robot side

open a new terminal

```bash
roscore
```

plugin ranger's (agilex) usb2can module and open a new terminal

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0
bash can_activate.sh can0 500000 "1-4.3:1.0"
# launch ranger control node
roslaunch ranger_bringup ranger_mini_v2.launch
```

open a new terminal and run paddle2ranger bridge node

```bash
source devel/setup.bash
rosrun teleop_bridge ranger_teleop_to_robot_paddle.py _input_angular_min:=-0.5 _input_angular_max:=0.5 _input_linear_min:=-0.2 _input_linear_max:=0.2  _angular_deadzone:=0.02 _linear_deadzone:=0.02 max_vel:=0.25 _max_angular_vel:=0.5
```
# How to run?

## 0. Prerequisite

- clone <piper_ros>, <piper_sdk> (need install), <ranger_ros>, <ugv_sdk> and install all dependencies
- setup master and slave
    - check ip addresses ```${master}``` and ```${slave}```
    - master PC:
        - add ```${master} ${master_name}``` to ```/etc/hosts```
        - add ```${slave} ${slave_name}``` to ```/etc/hosts```
        - add ```export ROS_HOSTNAME=${master_name}``` to ```~/.bashrc```
    - slave PC:
        - add ```${master} ${master_name}``` to ```/etc/hosts```
        - add ```${slave} ${slave_name}``` to ```/etc/hosts```
        - add ```export ROS_HOSTNAME=${slave_name}``` to ```~/.bashrc```
        - add ```export ROS_MASTER_URI=http://${master_name}:11311``` to ```~/.bashrc```

## 1. Build

```bash
<catkin build> or <catkin_make>
```

## 2. Teleoperation side

### 2.1 setup paddle in a new terminal

```bash
source devel/setup.bash
```

plugin damiao usb2can module

```bash
sudo chmod -R 777 /dev/ttyACM*
roslaunch dm_controllers load_dm_hw.launch
```

### 2.2 setup dual-arm in a new terminal

plugin piper's (agilex) usb2can modules

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0, 1-4.4:1.0
bash can_activate.sh can_left 1000000 "1-4.3:1.0"
bash can_activate.sh can_right 1000000 "1-4.4:1.0"
# launch piper teleop node
roslaunch teleop_bridge start_dual_teleop_piper.launch left_can_port:=can_left right_can_port:=can_right
```

## 3. Robot side

### 3.1 setup ranger control in a new terminal

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

### 3.2 setup paddle2ranger in a new terminal

```bash
source devel/setup.bash
rosrun teleop_bridge ranger_teleop_to_robot_paddle.py _input_angular_min:=-0.5 _input_angular_max:=0.5 _input_linear_min:=-0.2 _input_linear_max:=0.2  _angular_deadzone:=0.02 _linear_deadzone:=0.02 max_vel:=0.25 _max_angular_vel:=0.5
```

### 3.3 setup dual-arm control in a new terminal

plugin piper's (agilex) usb2can modules

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0, 1-4.4:1.0
bash can_activate.sh can_left 1000000 "1-4.3:1.0"
bash can_activate.sh can_right 1000000 "1-4.4:1.0"
# launch piper control node
roslaunch teleop_bridge start_dual_ctrl_piper.launch left_can_port:=can_left right_can_port:=can_right
```
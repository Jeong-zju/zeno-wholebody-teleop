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
- install realsense SDK
    following instructions on https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md and downgrade the version
    ```bash
    sudo apt-get install -y --allow-downgrades \
    librealsense2-dkms \
    librealsense2=2.55.1* \
    librealsense2-dev=2.55.1* \
    librealsense2-utils=2.55.1* \
    librealsense2-gl=2.55.1*
    ```
- setup realsense-ros
    ```bash
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd realsense-ros/
    git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
    ```
    change device PID to add support for D405
    ```cpp
    // const uint16_t RS405_PID        = 0x0b0c; // DS5U
    const uint16_t RS405_PID    = 0x0B5B; // DS5U
    ```


## 1. Build

```bash
<catkin build> or <catkin_make>
```

## 2. Teleoperation side

### 2.1 Setup paddle (damiao usb2can module)

```bash
source devel/setup.bash
cd src/zeno-wholebody-teleop/teleop_side/bash
bash setup_teleop.sh
```

This script sets permissions for damiao usb2can module.

### 2.2 Setup dual-arm CAN ports

Plugin piper's (agilex) usb2can modules and setup CAN ports manually:

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0, 1-4.4:1.0
bash can_activate.sh can_left 1000000 "1-4.3:1.0"
bash can_activate.sh can_right 1000000 "1-4.4:1.0"
```

### 2.3 Launch teleoperation nodes

After setup, launch all teleoperation nodes in a single command:

```bash
source devel/setup.bash
roslaunch teleop_bridge start_teleop_all.launch left_can_port:=can_left right_can_port:=can_right auto_enable:=true enable_paddle:=true enable_dual_arm:=true
```

**Parameters:**
- `left_can_port`: CAN port name for left arm (default: `can_left`)
- `right_can_port`: CAN port name for right arm (default: `can_right`)
- `auto_enable`: Auto enable motors (default: `true`)
- `enable_paddle`: Enable paddle (damiao) node (default: `true`)
- `enable_dual_arm`: Enable dual-arm teleop node (default: `true`)
- `gripper_val_mutiple`: Gripper value multiplier (default: `2`)
- `girpper_exist`: Whether gripper exists (default: `true`)

## 3. Robot side

### 3.1 Setup ranger CAN port

Plugin ranger's (agilex) usb2can module and setup CAN port manually:

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0
bash can_activate.sh can0 500000 "1-4.3:1.0"
```

### 3.2 Setup dual-arm CAN ports

Plugin piper's (agilex) usb2can modules and setup CAN ports manually:

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0, 1-4.4:1.0
bash can_activate.sh can_left 1000000 "1-8.3:1.0"
bash can_activate.sh can_right 1000000 "1-8.4:1.0"
```

### 3.3 Launch robot nodes

After setup, launch all robot nodes in a single command:

```bash
source devel/setup.bash
roslaunch piper_bridge start_robot_all.launch ranger_can_port:=can0 left_can_port:=can_left right_can_port:=can_right enable_ranger:=true enable_paddle2ranger:=true enable_dual_arm:=true enable_cameras:=true enable_rviz:=true camera_left_usb_port:=2-1 camera_right_usb_port:=2-8 camera_top_usb_port:=2-2
```

**Parameters:**

**CAN ports:**
- `ranger_can_port`: CAN port name for ranger (default: `can0`)
- `left_can_port`: CAN port name for left arm (default: `can_left`)
- `right_can_port`: CAN port name for right arm (default: `can_right`)

**Ranger:**
- `enable_ranger`: Enable ranger control node (default: `true`)
- `ranger_model`: Ranger model (default: `ranger_mini_v2`)
- `ranger_odom_frame`: Odometry frame (default: `odom`)
- `ranger_base_frame`: Base frame (default: `base_link`)
- `ranger_update_rate`: Update rate in Hz (default: `50`)
- `ranger_odom_topic_name`: Odometry topic name (default: `odom`)
- `ranger_publish_odom_tf`: Publish odometry TF (default: `false`)

**Paddle2Ranger:**
- `enable_paddle2ranger`: Enable paddle2ranger node (default: `true`)
- `input_angular_min`: Minimum angular input (default: `-0.5`)
- `input_angular_max`: Maximum angular input (default: `0.5`)
- `input_linear_min`: Minimum linear input (default: `-0.2`)
- `input_linear_max`: Maximum linear input (default: `0.2`)
- `angular_deadzone`: Angular deadzone (default: `0.02`)
- `linear_deadzone`: Linear deadzone (default: `0.02`)
- `max_vel`: Maximum velocity (default: `0.25`)
- `max_angular_vel`: Maximum angular velocity (default: `0.5`)

**Dual-arm control:**
- `enable_dual_arm`: Enable dual-arm control node (default: `true`)
- `auto_enable`: Auto enable motors (default: `true`)
- `gripper_val_mutiple`: Gripper value multiplier (default: `2`)
- `girpper_exist`: Whether gripper exists (default: `true`)

**Cameras:**
- `enable_cameras`: Enable camera nodes (default: `true`)
- `camera_left_usb_port`: USB port ID for left camera (default: `2-1`)
- `camera_right_usb_port`: USB port ID for right camera (default: `2-8`)
- `camera_top_usb_port`: USB port ID for top camera (default: `2-2`)
- `enable_rviz`: Enable RViz (default: `true`)

**Note:** Use command `rs-enumerate-devices` to find all connected cameras and get their USB port IDs from the `Physical Port` field. The output should contain a line like: `Physical Port: /sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/video4linux/video8`. Extract the `2-1` part and use it as the `usb_port_id` parameter.
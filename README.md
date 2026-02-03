# How to run?

## 0. Prerequisite

- clone <piper_ros>, <piper_sdk> (need install), <ranger_ros>, [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk.git) and install all dependencies
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
- install [pymlg](https://github.com/decargroup/pymlg) in your env (recommand python=3.9)
- install ```pip install -r requirements.txt``` (may report error, recommand to install manually for each package after running and locate missing package)
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
    change device PID to add support for D405 in file ```realsense-ros/realsense2_camera/include/constants.h```
    ```cpp
    // const uint16_t RS405_PID        = 0x0b0c; // DS5U
    const uint16_t RS405_PID    = 0x0B5B; // DS5U
    ```


## 1. Build

```bash
catkin_make
```

## 2. Teleoperation side

### 2.1 Setup paddle (damiao usb2can module)

```bash
sudo chmod 777 /dev/ttyACM0
```

### 2.2 Setup dual-arm CAN ports

Plugin piper's (agilex) usb2can modules and setup CAN ports manually:

```bash
source devel/setup.bash
# activate can port
cd <piper_ros>
bash find_all_can_port.sh # output should look like 1-4.3:1.0, 1-4.4:1.0
bash can_activate.sh can_left 1000000 "1-2:1.0"
bash can_activate.sh can_right 1000000 "1-1:1.0"
```

### 2.3 Launch teleoperation nodes

After setup, launch all teleoperation nodes in a single command:

```bash
source devel/setup.bash
roslaunch teleop_setup start_teleop_all.launch
```

**Parameters:**

To modify parameters, edit the launch file `teleop_setup/launch/start_teleop_all.launch` directly.

- `left_can_port`: CAN port name for left arm (default: `can_left`)
- `right_can_port`: CAN port name for right arm (default: `can_right`)
- `auto_enable`: Auto enable motors (default: `true`)
- `enable_paddle`: Enable paddle (damiao) node (default: `true`)
- `enable_paddle_haptic`: Enable paddle haptic feedback from lidar (default: `true`). When enabled, uses `paddle_haptic.yaml` for motor haptic config; when disabled, uses default `haptic.yaml`.
- `enable_dual_arm`: Enable dual-arm teleop node (default: `true`)
- `gripper_val_mutiple`: Gripper value multiplier (default: `2`)
- `girpper_exist`: Whether gripper exists (default: `true`)
- `enable_gravity_compensation`: Enable gravity compensation node (default: `true`)

**Haptic Configuration:**

The haptic parameters are configured in `teleop_setup/config/paddle_haptic.yaml`, which includes:
- Motor haptic parameters (x, y, z axis): `kp_default`, `kd_default`, `repulsive_force_threshold`, etc.
- Lidar-based haptic parameters: `scan_topic`, `r_min`, `r_far`, `weight_max`, `delta`, `filter_alpha`

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
roslaunch robot_setup start_robot_all.launch
```

**Parameters:**

To modify parameters, edit the launch file `robot_setup/launch/start_robot_all.launch` directly.

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
- Note: Paddle2Ranger parameters (input ranges, deadzones, max velocities) are configured in `bridge/config/paddle.yaml`

**Dual-arm control:**
- `enable_dual_arm`: Enable dual-arm control node (default: `true`)
- `auto_enable`: Auto enable motors (default: `true`)
- `gripper_val_mutiple`: Gripper value multiplier (default: `2`)
- `girpper_exist`: Whether gripper exists (default: `true`)

**Cameras:**
- `enable_camera_left`: Enable left camera node (default: `true`)
- `enable_camera_right`: Enable right camera node (default: `true`)
- `enable_camera_top`: Enable top camera node (default: `true`)
- `camera_left_usb_port`: USB port ID for left camera (default: `2-4.2`)
- `camera_right_usb_port`: USB port ID for right camera (default: `2-4.4`)
- `camera_top_usb_port`: USB port ID for top camera (default: `2-4.3`)
- `enable_rviz`: Enable RViz (default: `true`)
- `use_default_rviz`: Use default RViz config instead of piper_dual_robot_rviz (default: `false`)
- `enable_handeye_tf`: Enable hand-eye calibration TF publishing (default: `true`)

**LiDAR and Reachability:**
- `enable_lidar`: Enable LiDAR node (default: `true`)
- `enable_reachability`: Enable reachability mask computation (default: `true`)
- `enable_reachability_ctrl`: Enable reachability-based base control (default: `false`)
- `enable_lidar_force`: Enable LiDAR-based force feedback for ranger (default: `true`)

**Note:** Use command `rs-enumerate-devices` to find all connected cameras and get their USB port IDs from the `Physical Port` field. The output should contain a line like: `Physical Port: /sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/video4linux/video8`. Extract the `2-1` part and use it as the `usb_port_id` parameter.

## 4. Record Data

```bash
rosbag record -O demo_001.bag --bz2 -b 4096 \
/robot/arm_left/end_pose \
/robot/arm_right/end_pose \
/robot/arm_left/joint_states_single \
/robot/arm_right/joint_states_single \
/robot/arm_left/pos_cmd \
/robot/arm_right/pos_cmd \
/teleop/arm_left/end_pose \
/teleop/arm_right/end_pose \
/teleop/arm_left/joint_states_single \
/teleop/arm_right/joint_states_single \
/realsense_left/color/image_raw \
/realsense_left/color/camera_info \
/realsense_left/aligned_depth_to_color/image_raw \
/realsense_left/aligned_depth_to_color/camera_info \
/realsense_right/color/image_raw \
/realsense_right/color/camera_info \
/realsense_right/aligned_depth_to_color/image_raw \
/realsense_right/aligned_depth_to_color/camera_info \
/realsense_top/color/image_raw \
/realsense_top/color/camera_info \
/realsense_top/aligned_depth_to_color/image_raw \
/realsense_top/aligned_depth_to_color/camera_info \
(add anything you like)
```
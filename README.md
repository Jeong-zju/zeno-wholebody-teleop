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

## 5. Technical Details

### 5.1 Leader-Follower Architecture

This project adopts a **Leader-Follower (master-slave) teleoperation architecture**, enabling operators to control the Robot-side (Follower) manipulator through the Teleop-side (Leader) manipulator.

#### 5.1.1 System Architecture

The system consists of two independent manipulator systems:

- **Teleop Side (Leader/Master)**:
  - Topic namespace: `/teleop/arm_left/` and `/teleop/arm_right/`
  - Function: Manipulator directly operated by the operator, capturing the operator's motion intent
  - Configuration: Launched using `piper_dual_teleop.launch`, loads `piper_ctrl_mit_gravity.yaml` configuration

- **Robot Side (Follower/Slave)**:
  - Topic namespace: `/robot/arm_left/` and `/robot/arm_right/`
  - Function: Manipulator that executes actual tasks, following the Leader's motion
  - Configuration: Launched using `piper_dual_robot.launch`, loads `piper_ctrl_mit.yaml` configuration

#### 5.1.2 Communication Flow

Leader and Follower communicate through ROS topics:

```
Teleop Side (Leader)                 Robot Side (Follower)
─────────────────                    ─────────────────
joint_states_single  ──────────────> joint_pos_cmd
                                      (position command)
                                      
joint_states_single  ──────────────> joint_tor_cmd
                                      (torque command, for force feedback)
                                      
                    <──────────────  joint_states_single
                    (joint state feedback)
                    
                    <──────────────  joint_states_compensated
                    (gravity compensation torque)
```

### 5.2 MIT Control Mode

MIT control mode is a **hybrid position-velocity-torque control** method that allows simultaneous control of position, velocity, and torque, providing more flexible control capabilities.

#### 5.2.1 MIT Control Equation

The core formula of MIT control is:

$$
\tau_i = K_{p,i}\big(q_{i,\mathrm{des}}-q_i\big) + K_{d,i}\big(\dot q_{i,\mathrm{des}}-\dot q_i\big) + \tau_{i,\mathrm{ff}}
$$

#### 5.2.2 MIT Control Parameters

Key parameters:
- `enable_pos`, `enable_vel`, `enable_tor`: Enable position/velocity/torque control
- `kp`, `kd`: Proportional and derivative gains (per-joint or global)
- `torque_scale`: Torque scaling factor for operator input

**Gravity Compensation**:
When `enable_gravity=true`, the final torque is computed as:

$$\tau_{\text{final}} = -(\tau_{\text{cmd}} - \tau_{\text{gravity}}) \cdot \text{torque\_scale} + \tau_{\text{gravity}}$$

This ensures gravity is fully compensated while scaling the operator's input torque.

### 5.3 Gravity Compensation Technical Details

#### 5.3.1 Gravity Compensation Node

Gravity compensation is implemented by an independent node `piper_gravity_compensation_node.py`, which:

- **Subscribes to**: `/robot/arm_left/joint_states_single` and `/robot/arm_right/joint_states_single`
- **Publishes to**: `/robot/arm_left/joint_states_compensated` and `/robot/arm_right/joint_states_compensated`

#### 5.3.2 Gravity Compensation Calculation

Gravity compensation uses Pinocchio library to compute generalized gravity torques from URDF model:

$$\tau_{\text{gravity}} = \text{computeGeneralizedGravity}(\mathbf{q})$$

**Joint Scaling**:
- Joint1-3 (base joints): $\tau_{\text{comp}} = \tau_{\text{gravity}} / 4$ (reduced due to larger reduction ratios)
- Joint4-6 (wrist joints): $\tau_{\text{comp}} = \tau_{\text{gravity}}$ (unchanged)

### 5.4 Topic Mapping Summary

#### 5.4.1 Teleop Side Topic Flow

```
Operator moves Leader manipulator
    ↓
/teleop/arm_left/joint_states_single (publish)
    ↓
    ├─> /teleop/arm_left/joint_pos_cmd (self-subscribe, position control)
    ├─> /robot/arm_left/joint_pos_cmd (Robot side subscribes, position following)
    └─> /robot/arm_left/joint_tor_cmd (Robot side subscribes, force feedback)
    
/robot/arm_left/joint_states_single (Robot side publishes)
    ↓
    ├─> /teleop/arm_left/joint_tor_cmd (Teleop side subscribes, force feedback)
    └─> /robot/arm_left/joint_states_compensated (gravity compensation node subscribes)
    
/robot/arm_left/joint_states_compensated (gravity compensation node publishes)
    ↓
    └─> /teleop/arm_left/joint_states_compensated (Teleop side subscribes, gravity compensation)
```

#### 5.4.2 Robot Side Topic Flow

```
/robot/arm_left/joint_pos_cmd (receive position command)
    ↓
Robot side MIT control node
    ↓
CAN bus → Motor driver
    ↓
/robot/arm_left/joint_states_single (publish joint state)
    ↓
    ├─> Gravity compensation node
    ├─> Teleop side (force feedback)
    └─> Other monitoring nodes
```

### 5.5 Control Frequencies

- **Publish rate**: 100 Hz (`publish_rate`)
- **Control rate**: 50 Hz (`control_rate`)
- **Subscribe rate**: 50 Hz (`subscribe_rate`)

These frequencies ensure a balance between real-time performance and stability.

### 5.6 Paddle and Ranger Control

This section describes the paddle (input device) control system and its integration with ranger (mobile base) control, including velocity command fusion, intent computation, and LiDAR-driven haptic feedback.

#### 5.6.1 Paddle Input Device

The paddle is a 3-DOF input device (X: forward/backward, Y: left/right, Z: rotation) that maps motor positions to velocity commands through clamping, deadzone, and linear scaling. When angular velocity (Z) is non-zero, lateral velocity (Y) is set to 0 to avoid conflicts.

#### 5.6.2 Velocity Command Fusion

Paddle velocity and intent velocity are fused additively:

$$\mathbf{v}_{\text{fused}} = \mathbf{v}_{\text{paddle}} + \mathbf{v}_{\text{intent}}$$

If intent velocity is unavailable, only paddle velocity is used.

#### 5.6.3 Intent Velocity Computation

Intent velocity assists the operator when arms approach workspace limits. It's computed only for "stretched" arms (end-effector distance > threshold) with manipulability below threshold.

**Single Arm**:
$$\mathbf{d}_{\text{intent}} = w_{\text{target}} \cdot \hat{\mathbf{d}}_{\text{ee}} + w_{\text{grad}} \cdot \hat{\mathbf{d}}_{\text{grad}}$$

where $\hat{\mathbf{d}}_{\text{ee}}$ is the normalized end-effector direction and $\hat{\mathbf{d}}_{\text{grad}}$ is the manipulability gradient direction.

**Dual Arm** (weighted by manipulability deficit):
$$w_i = \frac{\max(\text{threshold} - m_i, \epsilon)}{\sum_j \max(\text{threshold} - m_j, \epsilon)}$$

$$\mathbf{d}_{\text{intent}} = \text{normalize}(w_{\text{left}} \cdot \mathbf{p}_{\text{left}} + w_{\text{right}} \cdot \mathbf{p}_{\text{right}})$$

Arms with lower manipulability (higher deficit) receive higher weights.

#### 5.6.4 LiDAR-Driven Haptic Feedback

LiDAR scans generate repulsive forces that create haptic feedback on the paddle to warn of nearby obstacles.

**Distance Weighting Function**:

$$w(r) = \begin{cases}
0 & r < r_{\min} - \delta \text{ or } r > r_{\text{far}} \\
w_{\max} \cdot (3t^2 - 2t^3) & r_{\min} - \delta \leq r < r_{\min} \text{ (smoothstep)} \\
w_{\max} \cdot (1 - \frac{r - r_{\min}}{r_{\text{far}} - r_{\min}})^2 & r_{\min} \leq r \leq r_{\text{far}} \text{ (quadratic)}
\end{cases}$$

where $t = \frac{r - (r_{\min} - \delta)}{\delta}$.

**Repulsive Force**:

$$\mathbf{F}_{\text{rep}} = -\frac{1}{N} \sum_{i=1}^{N} w(r_i) \cdot \hat{\mathbf{r}}_i$$

The force is low-pass filtered and applied to paddle motors as position offsets when exceeding threshold, creating a "virtual force field" effect.

#### 5.6.5 Topic Flow Summary

```
LiDAR Scan (/scan)
    ↓
paddle_haptic_client.py
    ├─> Compute repulsive forces
    ├─> Apply distance weighting
    ├─> Low-pass filter
    └─> Publish /repulsive_force_vector
         ↓
DmController (subscribes to /repulsive_force_vector)
    ├─> Apply haptic to X-axis motor
    ├─> Apply haptic to Y-axis motor
    └─> Z-axis: no haptic

Paddle Motors (/paddle/state)
    ↓
ranger_teleop_to_robot_paddle.py
    ├─> Map motor positions to velocities
    ├─> Apply deadzone and scaling
    └─> Fuse with intent velocity
         ↓
/robot/intent_vel (from manipulability_base_control_node)
    ↓
Fused velocity command (/cmd_vel)
    ↓
Ranger base control
```

This integrated system enables intuitive mobile base control with automatic assistance when arms approach workspace limits and haptic warnings for obstacle avoidance.
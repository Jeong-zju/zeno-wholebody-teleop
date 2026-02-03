#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ACT Evaluation ROS Node

Subscribes to:
- /robot/arm_left/joint_states_single
- /robot/arm_right/joint_states_single
- /realsense_left/color/image_raw/compressed
- /realsense_right/color/image_raw/compressed
- /realsense_top/color/image_raw/compressed

Publishes:
- /robot/arm_left/act_joint_cmd
- /robot/arm_right/act_joint_cmd
"""

import sys
import os
import rospy
import numpy as np
import cv2
import torch
import pickle
from threading import Lock
from sensor_msgs.msg import JointState, CompressedImage
from cv_bridge import CvBridge
from einops import rearrange

# Add act directory to path to import modules
# act_dir = os.path.join('/home/zeno/piper_ros/act')
# sys.path.insert(0, act_dir)

from constants import DT
from policy import ACTPolicy, CNNMLPPolicy
from utils import set_seed


class ACTEvalNode:
    def __init__(self):
        # Get ROS parameters (same as imitate_episodes.py)
        self.ckpt_dir = rospy.get_param('~ckpt_dir', '/home/zeno/piper_ros/act/ckpt/TriPilot-FF-P1-TabletopSort_kl10_cs30_bs16_lr1e-5_raw')
        self.task_name = rospy.get_param('~task_name', 'TriPilot-FF-P1-TabletopSort')
        self.policy_class = rospy.get_param('~policy_class', 'ACT')
        # self.use_qtor = rospy.get_param('~use_qtor', True)
        # self.use_lidar = rospy.get_param('~use_lidar', False)
        # self.mix = rospy.get_param('~mix', False)
        self.temporal_agg = rospy.get_param('~temporal_agg', False)
        self.seed = rospy.get_param('~seed', 1000)
        self.show_images = rospy.get_param('~show_images', True)  # Enable image display by default
        
        # ACT-specific parameters
        self.chunk_size = rospy.get_param('~chunk_size', 30)
        self.kl_weight = rospy.get_param('~kl_weight', 10)
        self.hidden_dim = rospy.get_param('~hidden_dim', 512)
        self.dim_feedforward = rospy.get_param('~dim_feedforward', 3200)
        
        # Get task configuration
        is_sim = self.task_name[:4] == 'sim_'
        if is_sim:
            from constants import SIM_TASK_CONFIGS
            task_config = SIM_TASK_CONFIGS[self.task_name]
        else:
            from constants import REAL_TASK_CONFIGS
            task_config = REAL_TASK_CONFIGS[self.task_name]
        
        self.camera_names = task_config['camera_names']
        self.state_dim = rospy.get_param('~state_dim', 14)
        
        # Fixed parameters
        lr_backbone = 1e-5
        backbone = 'resnet18'
        if self.policy_class == 'ACT':
            enc_layers = 4
            dec_layers = 7
            nheads = 8
            policy_config = {
                'lr': 1e-5,  # Not used during inference
                'num_queries': self.chunk_size,
                'kl_weight': self.kl_weight,
                'hidden_dim': self.hidden_dim,
                'dim_feedforward': self.dim_feedforward,
                'lr_backbone': lr_backbone,
                'backbone': backbone,
                'enc_layers': enc_layers,
                'dec_layers': dec_layers,
                'nheads': nheads,
                'camera_names': self.camera_names,
                'state_dim': self.state_dim,
                # 'use_lidar': self.use_lidar,
            }
        elif self.policy_class == 'CNNMLP':
            policy_config = {
                'lr': 1e-5,
                'lr_backbone': lr_backbone,
                'backbone': backbone,
                'num_queries': 1,
                'camera_names': self.camera_names,
            }
        else:
            raise NotImplementedError(f"Policy class {self.policy_class} not supported")
        
        # Load policy and stats
        set_seed(self.seed)
        self.policy = self._make_policy(self.policy_class, policy_config)
        ckpt_path = os.path.join(self.ckpt_dir, 'policy_best.ckpt')
        loading_status = self.policy.load_state_dict(torch.load(ckpt_path, map_location='cpu'))
        rospy.loginfo(f"Model loading status: {loading_status}")
        self.policy.cuda()
        self.policy.eval()
        rospy.loginfo(f'Loaded model from: {ckpt_path}')
        
        # Load dataset stats
        stats_path = os.path.join(self.ckpt_dir, 'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            self.stats = pickle.load(f)
        
        # Verify stats shapes and add debug info
        rospy.loginfo(f"Loaded normalization stats:")
        rospy.loginfo(f"  qpos_mean shape: {self.stats['qpos_mean'].shape}, mean: {self.stats['qpos_mean']}")
        rospy.loginfo(f"  qpos_std shape: {self.stats['qpos_std'].shape}, std: {self.stats['qpos_std']}")
        rospy.loginfo(f"  action_mean shape: {self.stats['action_mean'].shape}, mean: {self.stats['action_mean']}")
        rospy.loginfo(f"  action_std shape: {self.stats['action_std'].shape}, std: {self.stats['action_std']}")
        
        # Ensure stats are numpy arrays with correct shape
        for key in ['qpos_mean', 'qpos_std', 'action_mean', 'action_std']:
            if not isinstance(self.stats[key], np.ndarray):
                self.stats[key] = np.array(self.stats[key])
            # Ensure 1D array
            if self.stats[key].ndim == 0:
                self.stats[key] = self.stats[key].reshape(1)
            elif self.stats[key].ndim > 1:
                self.stats[key] = self.stats[key].flatten()
        
        # Image preprocessing
        # Note: Do NOT normalize here - ACTPolicy already does ImageNet normalization internally
        self.bridge = CvBridge()
        
        # Data storage with locks
        self.data_lock = Lock()
        self.joint_states_left = None
        self.joint_states_right = None
        self.images = {cam: None for cam in self.camera_names}
        self.images_bgr = {cam: None for cam in self.camera_names}  # Store BGR for display
        self.qtor_left = None
        self.qtor_right = None
        self.lidar_scan = None
        
        # Temporal aggregation setup
        self.query_frequency = self.chunk_size if not self.temporal_agg else 1
        if self.temporal_agg:
            self.num_queries = self.chunk_size
            self.all_time_actions = torch.zeros([10000, 10000 + self.num_queries, self.state_dim]).cuda()
        self.timestep = 0
        self.all_actions = None
        self.action_index = 0  # Index within current action chunk
        
        # Publishers
        self.pub_left = rospy.Publisher('/robot/arm_left/act_joint_cmd', JointState, queue_size=1)
        self.pub_right = rospy.Publisher('/robot/arm_right/act_joint_cmd', JointState, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/robot/arm_left/joint_states_single', JointState, 
                         self.joint_state_left_callback, queue_size=1)
        rospy.Subscriber('/robot/arm_right/joint_states_single', JointState,
                         self.joint_state_right_callback, queue_size=1)
        
        # Map camera names to topics
        camera_topic_map = {
            'left': '/realsense_left/color/image_raw/compressed',
            'right': '/realsense_right/color/image_raw/compressed',
            'top': '/realsense_top/color/image_raw/compressed'
        }
        
        for cam_name in self.camera_names:
            if cam_name in camera_topic_map:
                rospy.Subscriber(camera_topic_map[cam_name], CompressedImage,
                               lambda msg, name=cam_name: self.image_callback(msg, name), queue_size=1)
        
        rospy.loginfo("ACT Eval Node initialized")
        rospy.loginfo(f"Task: {self.task_name}, Policy: {self.policy_class}")
        rospy.loginfo(f"Cameras: {self.camera_names}")
        rospy.loginfo(f"Image display: {self.show_images}")
        
        # Control loop timer (10 Hz = 0.1 seconds period)
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
    
    def _make_policy(self, policy_class, policy_config):
        if policy_class == 'ACT':
            return ACTPolicy(policy_config)
        elif policy_class == 'CNNMLP':
            return CNNMLPPolicy(policy_config)
        else:
            raise NotImplementedError
    
    def _pre_process(self, qpos):
        """Preprocess qpos: pad if needed, handle mix parameter, then normalize."""
        # Ensure qpos is numpy array
        if not isinstance(qpos, np.ndarray):
            qpos = np.array(qpos)
        
        # Ensure shapes match for broadcasting
        qpos_mean = self.stats['qpos_mean']
        qpos_std = self.stats['qpos_std']
        
        # Check dimensions match
        if qpos.shape != qpos_mean.shape:
            rospy.logwarn_throttle(5.0, f"qpos shape mismatch: qpos={qpos.shape}, qpos_mean={qpos_mean.shape}")
            # Try to handle mismatch by flattening or reshaping
            if qpos.size == qpos_mean.size:
                qpos = qpos.flatten()
                qpos_mean = qpos_mean.flatten()
                qpos_std = qpos_std.flatten()
            else:
                rospy.logerr(f"qpos size mismatch: qpos={qpos.size}, qpos_mean={qpos_mean.size}")
                raise ValueError(f"qpos dimension mismatch: {qpos.shape} vs {qpos_mean.shape}")
        
        normalized = (qpos - qpos_mean) / qpos_std
        return normalized
    
    def _post_process(self, action):
        """Post-process action: pad if needed, denormalize, then remove padding if needed."""
        # Ensure action is numpy array
        if not isinstance(action, np.ndarray):
            action = np.array(action)
        
        # Ensure shapes match for broadcasting
        action_mean = self.stats['action_mean']
        action_std = self.stats['action_std']
        
        # Check dimensions match
        if action.shape != action_mean.shape:
            rospy.logwarn_throttle(5.0, f"action shape mismatch: action={action.shape}, action_mean={action_mean.shape}")
            # Try to handle mismatch by flattening or reshaping
            if action.size == action_mean.size:
                action = action.flatten()
                action_mean = action_mean.flatten()
                action_std = action_std.flatten()
            else:
                rospy.logerr(f"action size mismatch: action={action.size}, action_mean={action_mean.size}")
                raise ValueError(f"action dimension mismatch: {action.shape} vs {action_mean.shape}")
        
        action_denorm = action * action_std + action_mean
        return action_denorm
    
    def joint_state_left_callback(self, msg):
        with self.data_lock:
            self.joint_states_left = msg
            # if self.use_qtor and len(msg.effort) >= len(msg.position):
            #     self.qtor_left = np.array(msg.effort[:len(msg.position)], dtype=np.float32)
    
    def joint_state_right_callback(self, msg):
        with self.data_lock:
            self.joint_states_right = msg
            # if self.use_qtor and len(msg.effort) >= len(msg.position):
            #     self.qtor_right = np.array(msg.effort[:len(msg.position)], dtype=np.float32)
    
    def image_callback(self, msg, cam_name):
        """Decode compressed image and store."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is not None and img_bgr.size > 0:
                # Convert BGR to RGB for model
                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                with self.data_lock:
                    self.images[cam_name] = img_rgb
                    # Store BGR for display
                    self.images_bgr[cam_name] = img_bgr
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Error in image_callback for {cam_name}: {e}")
    
    def _get_image_tensor(self):
        """Get current images as tensor."""
        curr_images = []
        with self.data_lock:
            for cam_name in self.camera_names:
                if self.images[cam_name] is not None:
                    img = self.images[cam_name]
                    # Resize if needed (assuming model expects specific size)
                    # For now, use image as-is
                    curr_image = rearrange(img, 'h w c -> c h w')
                    curr_images.append(curr_image)
                else:
                    # Return None if images not ready
                    return None
        
        if len(curr_images) != len(self.camera_names):
            return None
        
        curr_image = np.stack(curr_images, axis=0)
        curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
        # Note: Do NOT normalize here - ACTPolicy.forward() already applies ImageNet normalization
        return curr_image
    
    
    def control_loop(self, event):
        """Main control loop - runs at DT frequency."""
        with self.data_lock:
            # Check if we have all required data
            if self.joint_states_left is None or self.joint_states_right is None:
                return
            
        # Check if all images are available
        if any(self.images[cam] is None for cam in self.camera_names):
            return
        
        # Display is handled by separate timer callback in main thread
        # No need to call display here
        
        # Get images as tensor
        curr_image = self._get_image_tensor()
        if curr_image is None:
            return
        
        # Process joint states
        with self.data_lock:
            qpos_left_np = np.array(self.joint_states_left.position, dtype=np.float32)
            qpos_right_np = np.array(self.joint_states_right.position, dtype=np.float32)
            
            # Combine left and right qpos first
            # For dual arm: typically 7 per arm (6 joints + 1 gripper) = 14 total
            qpos_combined = np.concatenate([qpos_left_np, qpos_right_np])
        
        # Preprocess qpos
        qpos = self._pre_process(qpos_combined)
        qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
        
        # Get qtor if needed
        # if self.use_qtor:
        #     with self.data_lock:
        #         if self.qtor_left is not None and self.qtor_right is not None:
        #             qtor_combined = np.concatenate([self.qtor_left, self.qtor_right])
        #             qtor = self._pre_process(qtor_combined)
        #             qtor = torch.from_numpy(qtor).float().cuda().unsqueeze(0)
        #         else:
        #             qtor = torch.zeros_like(qpos)
        # else:
        #     qtor = torch.zeros_like(qpos)
        
        # Query policy
        with torch.inference_mode():
            if self.policy_class == "ACT":
                if self.temporal_agg:
                    if self.timestep % self.query_frequency == 0:
                        all_actions = self.policy(qpos, curr_image)
                        self.all_time_actions[[self.timestep], self.timestep:self.timestep+self.num_queries] = all_actions
                    
                    actions_for_curr_step = self.all_time_actions[:, self.timestep]
                    actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                    actions_for_curr_step = actions_for_curr_step[actions_populated]
                    
                    if len(actions_for_curr_step) > 0:
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                    else:
                        return  # Not enough actions yet
                    
                    self.timestep += 1
                else:
                    # Non-temporal aggregation: query every query_frequency steps
                    # Use actions sequentially within each chunk
                    if self.action_index == 0 or self.all_actions is None:
                        # Query new action chunk
                        self.all_actions = self.policy(qpos, curr_image)
                        self.action_index = 0
                    
                    # Use action at current index within chunk
                    raw_action = self.all_actions[:, self.action_index]
                    
                    # Increment index and reset when reaching end of chunk
                    self.action_index += 1
                    if self.action_index >= self.query_frequency:
                        self.action_index = 0
                    
                    self.timestep += 1
            elif self.policy_class == "CNNMLP":
                raw_action = self.policy(qpos, curr_image)
            else:
                raise NotImplementedError
        
        # Post-process action
        raw_action = raw_action.squeeze(0).cpu().numpy()
        action = self._post_process(raw_action)
        
        # Split action into left and right
        # For dual arm: typically 14D (7 per arm) or 17D (3 base + 14 arm)
        # Remove base velocities if present (first 3 values for mobile)
        action_arm_only = action
        
        if len(action_arm_only) >= 14:
            # Dual arm: 7 joints per arm
            mid_point = len(action_arm_only) // 2
            action_left = action_arm_only[:mid_point]
            action_right = action_arm_only[mid_point:]
        elif len(action_arm_only) == 7:
            # Single arm mode: apply same action to both
            action_left = action_arm_only
            action_right = action_arm_only
        else:
            rospy.logwarn_throttle(5.0, f"Unexpected action dimension: {len(action)} (arm_only: {len(action_arm_only)})")
            return
        
        # Publish commands
        msg_left = JointState()
        msg_left.header.stamp = rospy.Time.now()
        msg_left.position = action_left.tolist()
        
        msg_right = JointState()
        msg_right.header.stamp = rospy.Time.now()
        msg_right.position = action_right.tolist()
        
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)


def main():
    rospy.init_node('act_eval_node', anonymous=True)
    node = ACTEvalNode()
    
    rate = rospy.Rate(30)  # 30 Hz for display
    
    while not rospy.is_shutdown():
        # Display images in main loop (not in callback)
        if node.show_images:
            with node.data_lock:
                for cam_name in node.camera_names:
                    if node.images_bgr[cam_name] is not None:
                        cv2.imshow(cam_name, node.images_bgr[cam_name])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        rate.sleep()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


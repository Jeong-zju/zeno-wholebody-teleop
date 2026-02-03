#!/usr/bin/env python3
"""
ROS Node for Image Rendering with Manipulability Visualization
===============================================================

This ROS node:
  1. Subscribes to camera_info, compressed image, and point cloud topics
  2. Aligns point cloud with RGB image
  3. Transforms point cloud to arm base frame using TF
  4. Computes manipulability for each point using trained MLP
  5. Renders image with color gradient overlay (red=low manip, blue=high manip)
  6. Publishes rendered image

Configuration via ROS parameters:
  ~model_path: Path to trained PyTorch model (model.pth)
  ~config_path: Path to model config (config.json)
  ~side: Side ('left', 'right', or 'top')
  ~camera_info_topic: Camera info topic (default: /realsense_{side}/color/camera_info)
  ~image_topic: Compressed image topic (default: /realsense_{side}/color/image_raw/compressed)
  ~pointcloud_topic: Point cloud topic (default: /realsense_{side}/depth/color/points)
  ~output_image_topic: Output rendered image topic (default: /realsense_{side}/color/image_rendered/compressed)
  ~device: PyTorch device ('cpu' or 'cuda')

Usage:
  rosrun piper_reachable_region img_render.py

Or in a launch file:
  <node pkg="piper_reachable_region" type="img_render.py" name="img_render">
    <param name="model_path" value="$(find piper_reachable_region)/model/model.pth" />
    <param name="config_path" value="$(find piper_reachable_region)/model/config.json" />
    <param name="side" value="left" />
    <param name="device" value="cuda" />
  </node>

Author: Teleop Team
"""

import json
import os
import sys
import traceback
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rospy
import torch
import tf2_ros
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2

# Import shared network definition
from network import ReachabilityMLP


class ImageRenderNode:
    """ROS node for rendering images with manipulability visualization."""

    def __init__(self):
        """Initialize the node: load model, set up TF, create subscribers/publishers."""
        rospy.loginfo("Initializing ImageRenderNode...")

        # Get parameters
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/zeno/piper_ros/src/zeno-wholebody-teleop/robot_side/piper_reachable_region/model/model.pth",
        )
        self.config_path = rospy.get_param(
            "~config_path",
            "/home/zeno/piper_ros/src/zeno-wholebody-teleop/robot_side/piper_reachable_region/model/config.json",
        )
        self.side = rospy.get_param("~side", "right")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", f"/realsense_{self.side}/color/camera_info")
        self.image_topic = rospy.get_param("~image_topic", f"/realsense_{self.side}/color/image_raw/compressed")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", f"/realsense_{self.side}/depth/color/points")
        self.output_image_topic = rospy.get_param(
            "~output_image_topic", f"/realsense_{self.side}/color/image_rendered/compressed"
        )
        device_str = rospy.get_param("~device", "cpu")

        # Frame names
        self.camera_frame = f"realsense_{self.side}_color_optical_frame"
        self.arm_base_frame = f"arm_{self.side}/base_link"

        rospy.loginfo(f"Model path: {self.model_path}")
        rospy.loginfo(f"Config path: {self.config_path}")
        rospy.loginfo(f"Side: {self.side}")
        rospy.loginfo(f"Camera info topic: {self.camera_info_topic}")
        rospy.loginfo(f"Image topic: {self.image_topic}")
        rospy.loginfo(f"Point cloud topic: {self.pointcloud_topic}")
        rospy.loginfo(f"Output image topic: {self.output_image_topic}")

        # Setup device
        if device_str == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device_str)
        rospy.loginfo(f"Using device: {self.device}")

        # Load config
        try:
            with open(self.config_path, "r") as f:
                self.config = json.load(f)
            rospy.loginfo("Config loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            raise

        # Extract normalization parameters
        self.xyz_min = np.array(self.config["normalization"]["xyz_min"])
        self.xyz_max = np.array(self.config["normalization"]["xyz_max"])
        self.manip_max = self.config["normalization"]["manip_max"]
        self.manip_min = self.config["normalization"]["manip_min"]

        # Load model
        try:
            model_config = self.config["model_architecture"]
            self.model = ReachabilityMLP(
                input_dim=model_config["input_dim"],
                hidden_dim=model_config["hidden_dim"],
                num_layers=model_config["num_layers"],
                output_manip=model_config["output_manip"],
            ).to(self.device)
            self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
            self.model.eval()
            rospy.loginfo("Model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            raise

        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.5)  # Wait for TF to be populated

        # Setup bridge
        self.bridge = CvBridge()

        # Store latest camera info
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Setup subscribers and publishers
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1
        )

        # Use message filters to synchronize image and point cloud
        self.image_sub = message_filters.Subscriber(self.image_topic, CompressedImage)
        self.pointcloud_sub = message_filters.Subscriber(self.pointcloud_topic, PointCloud2)

        # Synchronize with approximate time policy (allow 0.1s time difference)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.image_pointcloud_callback)

        # Publisher for rendered image
        self.image_pub = rospy.Publisher(self.output_image_topic, CompressedImage, queue_size=1)

        rospy.loginfo("ImageRenderNode initialized successfully")

    def camera_info_callback(self, msg):
        """Callback for camera info messages."""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D) if len(msg.D) > 0 else None
        rospy.loginfo("Camera info received")

    def normalize_xyz(self, xyz):
        """
        Normalize 3D positions to [-1, 1]^3 using stored bounding box.

        Args:
            xyz: (N, 3) array of 3D positions in base frame

        Returns:
            xyz_normalized: (N, 3) array normalized to [-1, 1]^3
        """
        xyz_center = (self.xyz_min + self.xyz_max) / 2
        xyz_half_size = (self.xyz_max - self.xyz_min) / 2
        xyz_norm = (xyz - xyz_center) / (xyz_half_size + 1e-8)
        return np.clip(xyz_norm, -1.0, 1.0)

    def denormalize_manip(self, m_norm):
        """
        Denormalize manipulability from normalized space to original scale.

        Args:
            m_norm: (N,) normalized manipulability scores

        Returns:
            m: (N,) denormalized manipulability
        """
        manip_range = self.manip_max - self.manip_min
        m = self.manip_min + m_norm * manip_range
        return m

    def get_transform(self, source_frame, target_frame, time):
        """
        Get transform from source to target frame.

        TF tree chain: 'realsense_{SIDE}_color_optical_frame' -> 'arm_{SIDE}/gripper_base' ->
                       'arm_{SIDE}/link6' -> 'arm_{SIDE}/link5' -> ... -> 'arm_{SIDE}/link1' -> 'arm_{SIDE}/base_link'
        Direction: child -> parent (TF stores parent -> child, so we need to invert)

        Args:
            source_frame: Source frame name (child)
            target_frame: Target frame name (parent)
            time: ROS time

        Returns:
            TransformStamped or None if not available
        """
        # Build the transform chain from child to parent
        chain = [
            f"realsense_{self.side}_color_optical_frame",
            f"arm_{self.side}/gripper_base",
            f"arm_{self.side}/link6",
            f"arm_{self.side}/link5",
            f"arm_{self.side}/link4",
            f"arm_{self.side}/link3",
            f"arm_{self.side}/link2",
            f"arm_{self.side}/link1",
            f"arm_{self.side}/base_link",
        ]

        # Find source and target in chain
        try:
            source_idx = chain.index(source_frame)
            target_idx = chain.index(target_frame)
        except ValueError:
            rospy.logwarn(f"Source {source_frame} or target {target_frame} not in expected chain")
            # Try direct lookup as fallback
            try:
                return self.tf_buffer.lookup_transform(target_frame, source_frame, time, timeout=rospy.Duration(0.1))
            except:
                return None

        # If source is after target in chain, we need to go backwards
        if source_idx > target_idx:
            rospy.logwarn(
                f"Source {source_frame} (idx {source_idx}) is after target {target_frame} (idx {target_idx}) in chain"
            )
            return None

        # Build transform by composing transforms along the chain
        # TF stores transforms from parent to child, so we need to invert them
        accumulated_transform = None

        for i in range(source_idx, target_idx):
            child_frame = chain[i]
            parent_frame = chain[i + 1]

            try:
                # Lookup transform from parent to child (as stored in TF)
                parent_to_child = self.tf_buffer.lookup_transform(
                    child_frame, parent_frame, time, timeout=rospy.Duration(0.1)
                )

                # Invert to get child to parent transform
                child_to_parent = self.invert_transform(parent_to_child)

                # Compose with accumulated transform
                if accumulated_transform is None:
                    accumulated_transform = child_to_parent
                else:
                    accumulated_transform = self.compose_transforms(accumulated_transform, child_to_parent)

            except Exception as e:
                rospy.logwarn(f"Failed to get transform from {child_frame} to {parent_frame}: {e}")
                return None

        return accumulated_transform

    def invert_transform(self, transform):
        """Invert a TransformStamped transform."""
        from tf2_ros import TransformStamped

        # Get transform components
        t = transform.transform
        trans = np.array([t.translation.x, t.translation.y, t.translation.z])
        rot = np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

        # Build transform matrix
        R = Rotation.from_quat(rot).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = trans

        # Invert
        T_inv = np.linalg.inv(T)

        # Create inverted transform
        result = TransformStamped()
        result.header.frame_id = transform.child_frame_id  # Swap frames
        result.child_frame_id = transform.header.frame_id
        result.header.stamp = transform.header.stamp

        result.transform.translation.x = T_inv[0, 3]
        result.transform.translation.y = T_inv[1, 3]
        result.transform.translation.z = T_inv[2, 3]

        rot_inv = Rotation.from_matrix(T_inv[:3, :3]).as_quat()
        result.transform.rotation.x = rot_inv[0]
        result.transform.rotation.y = rot_inv[1]
        result.transform.rotation.z = rot_inv[2]
        result.transform.rotation.w = rot_inv[3]

        return result

    def compose_transforms(self, t1, t2):
        """
        Compose two TransformStamped transforms.

        If t1: A -> B, t2: B -> C, then result: A -> C
        """
        from tf2_ros import TransformStamped

        # Convert to matrices
        t1_trans = np.array([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z])
        t1_rot = np.array(
            [t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w]
        )
        t1_mat = np.eye(4)
        t1_mat[:3, :3] = Rotation.from_quat(t1_rot).as_matrix()
        t1_mat[:3, 3] = t1_trans

        t2_trans = np.array([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z])
        t2_rot = np.array(
            [t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w]
        )
        t2_mat = np.eye(4)
        t2_mat[:3, :3] = Rotation.from_quat(t2_rot).as_matrix()
        t2_mat[:3, 3] = t2_trans

        # Compose: result = t2 @ t1 (A -> B -> C)
        result_mat = t2_mat @ t1_mat

        # Convert back to TransformStamped
        result = TransformStamped()
        result.header.frame_id = t1.header.frame_id  # Start frame of t1
        result.child_frame_id = t2.child_frame_id  # End frame of t2
        result.header.stamp = t1.header.stamp

        result.transform.translation.x = result_mat[0, 3]
        result.transform.translation.y = result_mat[1, 3]
        result.transform.translation.z = result_mat[2, 3]

        result_quat = Rotation.from_matrix(result_mat[:3, :3]).as_quat()
        result.transform.rotation.x = result_quat[0]
        result.transform.rotation.y = result_quat[1]
        result.transform.rotation.z = result_quat[2]
        result.transform.rotation.w = result_quat[3]

        return result

    def project_points_to_image(self, points_3d, camera_matrix, dist_coeffs=None, image_shape=None):
        """
        Project 3D points to 2D image coordinates.

        Args:
            points_3d: (N, 3) array of 3D points in camera frame
            camera_matrix: (3, 3) camera intrinsic matrix
            dist_coeffs: Distortion coefficients (optional)
            image_shape: (height, width) tuple for image bounds

        Returns:
            points_2d: (N, 2) array of 2D image coordinates
            valid_mask: (N,) boolean array indicating valid projections
        """
        if dist_coeffs is None or len(dist_coeffs) == 0:
            dist_coeffs = np.zeros(5)
        elif len(dist_coeffs) < 5:
            dist_coeffs = np.pad(dist_coeffs, (0, 5 - len(dist_coeffs)), "constant")

        # Project points using camera intrinsics
        points_hom = points_3d.copy()
        points_hom[:, 2] = np.clip(points_hom[:, 2], 1e-6, None)  # Avoid division by zero

        # Project to normalized coordinates
        points_2d_norm = points_hom[:, :2] / points_hom[:, 2:3]

        # Apply distortion if needed
        if np.any(dist_coeffs != 0):
            # Use cv2.projectPoints for distortion
            points_2d, _ = cv2.projectPoints(
                points_3d.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), camera_matrix, dist_coeffs
            )
            points_2d = points_2d.reshape(-1, 2)
        else:
            # Simple projection
            points_2d = (camera_matrix[:2, :2] @ points_2d_norm.T).T + camera_matrix[:2, 2]

        # Check validity
        if image_shape is not None:
            h, w = image_shape
        else:
            h, w = 480, 640  # Default

        valid_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < w)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < h)
            & (points_3d[:, 2] > 0)  # Points in front of camera
        )

        return points_2d, valid_mask

    def image_pointcloud_callback(self, image_msg, pointcloud_msg):
        """
        Synchronized callback for image and point cloud messages.

        Args:
            image_msg: CompressedImage message
            pointcloud_msg: PointCloud2 message
        """
        try:
            # Check if camera info is available
            if self.camera_info is None or self.camera_matrix is None:
                rospy.logwarn_throttle(1.0, "Camera info not available yet")
                return

            # Decode image
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            except Exception as e1:
                # Fallback: decode compressed image directly with cv2
                try:
                    np_arr = np.frombuffer(image_msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if cv_image is None:
                        rospy.logwarn("Failed to decode compressed image")
                        return
                except Exception as e2:
                    rospy.logwarn(f"Failed to decode image: {e1}, {e2}")
                    return

            h_img, w_img = cv_image.shape[:2]

            # Extract point cloud
            points_list = list(point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=False))

            if len(points_list) == 0:
                # Publish original image if no points
                self.publish_image(cv_image, image_msg.header)
                return

            # Convert to numpy array
            points_camera = []
            for p in points_list:
                if p[0] is not None and p[1] is not None and p[2] is not None:
                    if not (np.isnan(p[0]) or np.isnan(p[1]) or np.isnan(p[2])):
                        if not (np.isinf(p[0]) or np.isinf(p[1]) or np.isinf(p[2])):
                            points_camera.append([p[0], p[1], p[2]])

            if len(points_camera) == 0:
                self.publish_image(cv_image, image_msg.header)
                return

            points_camera = np.array(points_camera, dtype=np.float32)

            # Transform point cloud to arm base frame
            transform = self.get_transform(self.camera_frame, self.arm_base_frame, pointcloud_msg.header.stamp)

            if transform is None:
                rospy.logwarn_throttle(1.0, "No transform available")
                self.publish_image(cv_image, image_msg.header)
                return

            # Apply transform
            t = transform.transform
            trans = np.array([t.translation.x, t.translation.y, t.translation.z])
            rot = np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

            # Build transform matrix
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(rot).as_matrix()
            T[:3, 3] = trans

            # Transform all points at once
            points_hom = np.ones((len(points_camera), 4))
            points_hom[:, :3] = points_camera
            points_base_hom = (T @ points_hom.T).T
            points_base = points_base_hom[:, :3].astype(np.float32)

            # Compute manipulability - following compute_manipulability_from_model pattern
            # Normalize position
            xyz_norm = self.normalize_xyz(points_base)

            # Query model
            with torch.no_grad():
                xyz_tensor = torch.from_numpy(xyz_norm).float().to(self.device)
                _, m_pred = self.model(xyz_tensor)

                if m_pred is not None:
                    # Get normalized manipulability directly from model output
                    m_pred_norm = m_pred.squeeze().cpu().numpy()
                    # Denormalize manipulability to get actual value
                    m_pred_denorm = self.denormalize_manip(m_pred_norm)
                    # For visualization, normalize the denormalized value to [0, 1]
                    manip_range = self.manip_max - self.manip_min
                    manip_normalized = (m_pred_denorm - self.manip_min) / (manip_range + 1e-8)
                else:
                    m_pred_norm = np.zeros(len(points_base))
                    manip_normalized = np.zeros(len(points_base))

            # Handle invalid points (set manip to 0)
            manip_normalized = np.where(np.isfinite(manip_normalized), manip_normalized, 0.0)

            # Filter: discard points with m_pred_norm < 1
            manip_mask = m_pred_norm >= 0.3

            if not np.any(manip_mask):
                # No points with manip >= 1, publish original image
                self.publish_image(cv_image, image_msg.header)
                return

            # Filter points, coordinates, and manipulability values
            points_camera = points_camera[manip_mask]
            points_base = points_base[manip_mask]
            m_pred_norm = m_pred_norm[manip_mask]
            manip_normalized = manip_normalized[manip_mask]

            # Clip to [0, 1] range for visualization
            manip_normalized = np.clip(manip_normalized, 0.0, 1.0)

            # Project points to image
            points_2d, valid_mask = self.project_points_to_image(
                points_camera, self.camera_matrix, self.dist_coeffs, image_shape=(h_img, w_img)
            )

            # Create color overlay based on manipulability
            # Color mapping: red (low manip) -> blue (high manip)
            manip_map = np.zeros((h_img, w_img), dtype=np.float32)
            count_map = np.zeros((h_img, w_img), dtype=np.int32)

            # Accumulate manipulability values for each pixel
            for i in range(len(points_2d)):
                if valid_mask[i]:
                    u, v = int(np.round(points_2d[i, 0])), int(np.round(points_2d[i, 1]))
                    if 0 <= u < w_img and 0 <= v < h_img:
                        manip_val = manip_normalized[i]
                        manip_map[v, u] += manip_val
                        count_map[v, u] += 1

            # Average overlapping points
            valid_pixels = count_map > 0
            manip_map[valid_pixels] /= count_map[valid_pixels]

            # Convert manipulability [0, 1] to hue [0, 120] (red to blue)
            # In OpenCV HSV, hue range is [0, 180]
            hue_map = 120.0 * (1.0 - manip_map)  # Low manip -> red (0°), high manip -> blue (120°)
            saturation_map = np.ones_like(manip_map) * 1.0
            value_map = np.ones_like(manip_map) * 1.0

            # Convert HSV to BGR
            hsv_overlay = np.zeros((h_img, w_img, 3), dtype=np.uint8)
            hsv_overlay[:, :, 0] = np.clip(hue_map, 0, 180).astype(np.uint8)
            hsv_overlay[:, :, 1] = (saturation_map * 255).astype(np.uint8)
            hsv_overlay[:, :, 2] = (value_map * 255).astype(np.uint8)

            color_overlay_bgr = cv2.cvtColor(hsv_overlay, cv2.COLOR_HSV2BGR)

            # Blend original image with color overlay
            alpha_map = np.zeros((h_img, w_img), dtype=np.float32)
            alpha_map[valid_pixels] = 0.6  # 60% opacity for overlay

            alpha_3d = np.stack([alpha_map] * 3, axis=2)
            result_image = (
                cv_image.astype(np.float32) * (1.0 - alpha_3d) + color_overlay_bgr.astype(np.float32) * alpha_3d
            ).astype(np.uint8)

            # Publish rendered image
            self.publish_image(result_image, image_msg.header)

        except Exception as e:
            rospy.logerr(f"Error in image_pointcloud_callback: {e}")
            rospy.logerr(traceback.format_exc())

    def publish_image(self, cv_image, header):
        """
        Publish rendered image as compressed image.

        Args:
            cv_image: OpenCV image (BGR)
            header: ROS header from original image
        """
        try:
            # Convert to compressed image
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
            compressed_msg.header = header
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            rospy.logwarn(f"Failed to publish image: {e}")


def main():
    """Main entry point for the ROS node."""
    rospy.init_node("img_render_node", anonymous=True)

    try:
        node = ImageRenderNode()
        rospy.loginfo("Spinning...")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze rosbag statistics for a given task.

Calculates for each subfolder (method) in the dataset:
- Average completion time
- Average minimum distance (from /weighted_pointcloud, distances > 0.40)
- Linear jerk RMS and Angular jerk RMS (from /ranger_base_node/odom)
- Joint effort statistics (from left and right arm joint states):
  - Left arm: /robot/arm_left/joint_states_single
  - Right arm: /robot/arm_right/joint_states_single
  - Combined: sum of left and right arm statistics
  - Max, min, mean effort for each joint
  - Mechanical energy consumption (integral of |torque * velocity| dt)
  - Torque-time energy (integral of |torque| dt, proportional to current consumption)
  - Average power
"""

import os
import sys
import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2, JointState
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
import rosbag
from tqdm import tqdm
from collections import defaultdict


def timestamp_to_float(stamp):
    """Convert ROS timestamp to float seconds."""
    return stamp.secs + stamp.nsecs * 1e-9


def read_pointcloud_distances(bag_path):
    """Read /weighted_pointcloud topic and extract distances."""
    topic = "/weighted_pointcloud"
    distances = []

    try:
        bag = rosbag.Bag(bag_path, "r")

        # Check if topic exists
        topics = bag.get_type_and_topic_info()[1].keys()
        if topic not in topics:
            bag.close()
            return None

        # Get message count for progress bar
        topic_info = bag.get_type_and_topic_info()[1]
        if topic in topic_info:
            msg_count = topic_info[topic].message_count
        else:
            msg_count = 0

        bag_name = os.path.basename(bag_path)
        for _, msg, _ in tqdm(
            bag.read_messages(topics=[topic]),
            total=msg_count,
            desc=f"      Reading pointcloud ({bag_name})",
            leave=False,
        ):

            # Extract points from PointCloud2
            # Try to get distance from intensity field first, then calculate from xyz
            try:
                # Try to read intensity field if available
                points_with_intensity = list(
                    point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=False)
                )
                if len(points_with_intensity) > 0 and points_with_intensity[0][3] is not None:
                    # Use intensity as distance
                    point_distances = np.array([p[3] for p in points_with_intensity if p[3] is not None])
                else:
                    # Calculate from xyz
                    points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                    if len(points) == 0:
                        continue
                    points_array = np.array(points)
                    # Calculate 2D distances from origin (x, y only, as in lidar_force_ranger.py)
                    point_distances = np.sqrt(points_array[:, 0] ** 2 + points_array[:, 1] ** 2)
            except:
                # Fallback: calculate from xyz
                points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                if len(points) == 0:
                    continue
                points_array = np.array(points)
                # Calculate 2D distances from origin (x, y only, as in lidar_force_ranger.py)
                point_distances = np.sqrt(points_array[:, 0] ** 2 + points_array[:, 1] ** 2)

            # Filter distances > 0.40
            filtered_distances = point_distances[point_distances > 0.40]

            if len(filtered_distances) > 0:
                # Get minimum distance for this pointcloud
                min_dist = np.min(filtered_distances)
                distances.append(min_dist)

        bag.close()

        if len(distances) == 0:
            return None

        return np.array(distances)

    except Exception as e:
        print(f"Error reading pointcloud from {bag_path}: {e}")
        return None


def calculate_jerk_from_odom(bag_path):
    """Read /ranger_base_node/odom and calculate RMS (Root Mean Square) for linear and angular jerk separately."""
    topic = "/ranger_base_node/odom"

    try:
        bag = rosbag.Bag(bag_path, "r")

        # Check if topic exists
        topics = bag.get_type_and_topic_info()[1].keys()
        if topic not in topics:
            bag.close()
            return None

        # Get message count for progress bar
        topic_info = bag.get_type_and_topic_info()[1]
        if topic in topic_info:
            msg_count = topic_info[topic].message_count
        else:
            msg_count = 0

        timestamps = []
        linear_velocities = []
        angular_velocities = []

        bag_name = os.path.basename(bag_path)
        for _, msg, t in tqdm(
            bag.read_messages(topics=[topic]), total=msg_count, desc=f"      Reading odom ({bag_name})", leave=False
        ):

            timestamp = timestamp_to_float(msg.header.stamp) if hasattr(msg, "header") else timestamp_to_float(t)
            timestamps.append(timestamp)

            # Extract linear and angular velocities
            linear_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            angular_vel = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

            linear_velocities.append(linear_vel)
            angular_velocities.append(angular_vel)

        bag.close()

        if len(timestamps) < 3:  # Need at least 3 points to calculate jerk
            return None

        timestamps = np.array(timestamps)
        linear_velocities = np.array(linear_velocities)
        angular_velocities = np.array(angular_velocities)

        # Sort by timestamp
        sort_idx = np.argsort(timestamps)
        timestamps = timestamps[sort_idx]
        linear_velocities = linear_velocities[sort_idx]
        angular_velocities = angular_velocities[sort_idx]

        # Calculate time differences
        dt = np.diff(timestamps)
        dt = np.clip(dt, 1e-6, None)  # Avoid division by zero

        # Calculate acceleration (derivative of velocity)
        linear_acc = np.diff(linear_velocities, axis=0) / dt[:, np.newaxis]
        angular_acc = np.diff(angular_velocities, axis=0) / dt[:, np.newaxis]

        if len(linear_acc) < 2:  # Need at least 2 acceleration points to calculate jerk
            return None

        # Calculate jerk (derivative of acceleration)
        dt_acc = dt[:-1]  # Time differences for acceleration
        dt_acc = np.clip(dt_acc, 1e-6, None)

        linear_jerk = np.diff(linear_acc, axis=0) / dt_acc[:, np.newaxis]
        angular_jerk = np.diff(angular_acc, axis=0) / dt_acc[:, np.newaxis]

        # Calculate RMS (Root Mean Square) for linear and angular jerk separately
        # RMS = sqrt(mean(sum(jerk^2)))
        linear_jerk_rms = np.sqrt(np.mean(np.sum(linear_jerk**2, axis=1)))
        angular_jerk_rms = np.sqrt(np.mean(np.sum(angular_jerk**2, axis=1)))

        return {"linear_jerk_rms": linear_jerk_rms, "angular_jerk_rms": angular_jerk_rms}

    except Exception as e:
        print(f"Error calculating jerk from {bag_path}: {e}")
        import traceback

        traceback.print_exc()
        return None


def read_joint_states_effort(bag_path, topic="/joint_states"):
    """Read joint states topic and extract effort (torque) data.

    Args:
        bag_path: Path to the bag file
        topic: Topic name for joint states (default: /joint_states)

    Returns:
        Dictionary containing:
        - timestamps: array of timestamps
        - joint_names: list of joint names
        - efforts: 2D array of efforts [time, joint]
        - positions: 2D array of positions [time, joint]
        - velocities: 2D array of velocities [time, joint]
    """
    try:
        bag = rosbag.Bag(bag_path, "r")

        # Check if topic exists
        topics = bag.get_type_and_topic_info()[1].keys()
        if topic not in topics:
            bag.close()
            return None

        # Get message count for progress bar
        topic_info = bag.get_type_and_topic_info()[1]
        if topic in topic_info:
            msg_count = topic_info[topic].message_count
        else:
            msg_count = 0

        timestamps = []
        efforts_list = []
        positions_list = []
        velocities_list = []
        joint_names = None

        bag_name = os.path.basename(bag_path)
        for _, msg, t in tqdm(
            bag.read_messages(topics=[topic]),
            total=msg_count,
            desc=f"      Reading joint states ({bag_name})",
            leave=False,
        ):

            timestamp = (
                timestamp_to_float(msg.header.stamp)
                if hasattr(msg, "header") and msg.header.stamp.secs > 0
                else timestamp_to_float(t)
            )

            # Get joint names from first message
            if joint_names is None:
                joint_names = list(msg.name)

            # Extract effort data
            if len(msg.effort) > 0:
                timestamps.append(timestamp)
                efforts_list.append(list(msg.effort))
                positions_list.append(list(msg.position) if len(msg.position) > 0 else [0.0] * len(msg.effort))
                velocities_list.append(list(msg.velocity) if len(msg.velocity) > 0 else [0.0] * len(msg.effort))

        bag.close()

        if len(timestamps) == 0 or joint_names is None:
            return None

        # Convert to numpy arrays
        timestamps = np.array(timestamps)
        efforts = np.array(efforts_list)
        positions = np.array(positions_list)
        velocities = np.array(velocities_list)

        # Sort by timestamp
        sort_idx = np.argsort(timestamps)
        timestamps = timestamps[sort_idx]
        efforts = efforts[sort_idx]
        positions = positions[sort_idx]
        velocities = velocities[sort_idx]

        return {
            "timestamps": timestamps,
            "joint_names": joint_names,
            "efforts": efforts,
            "positions": positions,
            "velocities": velocities,
        }

    except Exception as e:
        print(f"Error reading joint states from {bag_path}: {e}")
        import traceback

        traceback.print_exc()
        return None


def calculate_effort_statistics(joint_data):
    """Calculate effort statistics including energy consumption.

    Args:
        joint_data: Dictionary from read_joint_states_effort()

    Returns:
        Dictionary containing:
        - effort_max: max effort for each joint
        - effort_min: min effort for each joint
        - effort_mean: mean effort for each joint
        - effort_abs_max: max absolute effort for each joint
        - energy_mechanical: mechanical energy (integral of |τ * ω| dt) for each joint
        - energy_torque_time: simplified energy (integral of |τ| dt) for each joint
        - total_energy_mechanical: sum of mechanical energy across all joints
        - total_energy_torque_time: sum of torque-time energy across all joints
    """
    if joint_data is None:
        return None

    timestamps = joint_data["timestamps"]
    efforts = joint_data["efforts"]
    velocities = joint_data["velocities"]
    joint_names = joint_data["joint_names"]

    num_joints = efforts.shape[1]

    # Basic statistics
    effort_max = np.max(efforts, axis=0)
    effort_min = np.min(efforts, axis=0)
    effort_mean = np.mean(efforts, axis=0)
    effort_abs_max = np.max(np.abs(efforts), axis=0)
    effort_std = np.std(efforts, axis=0)

    # Calculate time differences for integration
    dt = np.diff(timestamps)
    dt = np.clip(dt, 1e-6, None)  # Avoid division by zero

    # Mechanical energy: E = ∫|τ * ω| dt (absolute value to count all work)
    # Use trapezoidal integration
    power = np.abs(efforts[:-1] * velocities[:-1])  # |τ * ω|
    energy_mechanical = np.sum(power * dt[:, np.newaxis], axis=0)

    # Simplified energy: E = ∫|τ| dt (torque-time integral, proportional to current consumption)
    torque_abs = np.abs(efforts[:-1])
    energy_torque_time = np.sum(torque_abs * dt[:, np.newaxis], axis=0)

    # Total energy across all joints
    total_energy_mechanical = np.sum(energy_mechanical)
    total_energy_torque_time = np.sum(energy_torque_time)

    # Duration
    duration = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0

    # Average power
    avg_power_mechanical = total_energy_mechanical / duration if duration > 0 else 0
    avg_power_torque_time = total_energy_torque_time / duration if duration > 0 else 0

    return {
        "joint_names": joint_names,
        "effort_max": effort_max,
        "effort_min": effort_min,
        "effort_mean": effort_mean,
        "effort_abs_max": effort_abs_max,
        "effort_std": effort_std,
        "energy_mechanical": energy_mechanical,
        "energy_torque_time": energy_torque_time,
        "total_energy_mechanical": total_energy_mechanical,
        "total_energy_torque_time": total_energy_torque_time,
        "avg_power_mechanical": avg_power_mechanical,
        "avg_power_torque_time": avg_power_torque_time,
        "duration": duration,
    }


def merge_arm_effort_statistics(left_stats, right_stats):
    """Merge left and right arm effort statistics.

    Args:
        left_stats: Effort statistics for left arm (from calculate_effort_statistics)
        right_stats: Effort statistics for right arm (from calculate_effort_statistics)

    Returns:
        Combined statistics with summed energy/power values
    """
    if left_stats is None and right_stats is None:
        return None
    if left_stats is None:
        return right_stats
    if right_stats is None:
        return left_stats

    # Merge joint names
    joint_names = left_stats["joint_names"] + right_stats["joint_names"]

    # Sum energy values
    total_energy_mechanical = left_stats["total_energy_mechanical"] + right_stats["total_energy_mechanical"]
    total_energy_torque_time = left_stats["total_energy_torque_time"] + right_stats["total_energy_torque_time"]

    # Sum power values
    avg_power_mechanical = left_stats["avg_power_mechanical"] + right_stats["avg_power_mechanical"]
    avg_power_torque_time = left_stats["avg_power_torque_time"] + right_stats["avg_power_torque_time"]

    # Concatenate per-joint statistics
    effort_max = np.concatenate([left_stats["effort_max"], right_stats["effort_max"]])
    effort_min = np.concatenate([left_stats["effort_min"], right_stats["effort_min"]])
    effort_mean = np.concatenate([left_stats["effort_mean"], right_stats["effort_mean"]])
    effort_abs_max = np.concatenate([left_stats["effort_abs_max"], right_stats["effort_abs_max"]])
    effort_std = np.concatenate([left_stats["effort_std"], right_stats["effort_std"]])
    energy_mechanical = np.concatenate([left_stats["energy_mechanical"], right_stats["energy_mechanical"]])
    energy_torque_time = np.concatenate([left_stats["energy_torque_time"], right_stats["energy_torque_time"]])

    # Use average duration
    duration = (left_stats["duration"] + right_stats["duration"]) / 2

    return {
        "joint_names": joint_names,
        "effort_max": effort_max,
        "effort_min": effort_min,
        "effort_mean": effort_mean,
        "effort_abs_max": effort_abs_max,
        "effort_std": effort_std,
        "energy_mechanical": energy_mechanical,
        "energy_torque_time": energy_torque_time,
        "total_energy_mechanical": total_energy_mechanical,
        "total_energy_torque_time": total_energy_torque_time,
        "avg_power_mechanical": avg_power_mechanical,
        "avg_power_torque_time": avg_power_torque_time,
        "duration": duration,
    }


def plot_effort_curves(joint_data, output_path, title=None):
    """Plot effort curves for all joints.

    Args:
        joint_data: Dictionary from read_joint_states_effort()
        output_path: Path to save the plot
        title: Optional title for the plot
    """
    if joint_data is None:
        return

    timestamps = joint_data["timestamps"]
    efforts = joint_data["efforts"]
    joint_names = joint_data["joint_names"]

    # Normalize time to start from 0
    time = timestamps - timestamps[0]

    num_joints = len(joint_names)

    # Create subplots
    fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2 * num_joints), sharex=True)

    if num_joints == 1:
        axes = [axes]

    for i, (ax, name) in enumerate(zip(axes, joint_names)):
        ax.plot(time, efforts[:, i], linewidth=0.8)
        ax.set_ylabel(f"{name}\nEffort (Nm)")
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color="k", linewidth=0.5)

        # Add statistics text
        max_val = np.max(efforts[:, i])
        min_val = np.min(efforts[:, i])
        mean_val = np.mean(efforts[:, i])
        ax.text(
            0.98,
            0.95,
            f"max={max_val:.2f}, min={min_val:.2f}, mean={mean_val:.2f}",
            transform=ax.transAxes,
            ha="right",
            va="top",
            fontsize=8,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        )

    axes[-1].set_xlabel("Time (s)")

    if title:
        fig.suptitle(title, fontsize=12)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"      Effort plot saved to: {output_path}")


def plot_dual_arm_effort_curves(left_data, right_data, output_path, title=None):
    """Plot effort curves comparing left and right arms in the same figure.

    Args:
        left_data: Dictionary from read_joint_states_effort() for left arm
        right_data: Dictionary from read_joint_states_effort() for right arm
        output_path: Path to save the plot
        title: Optional title for the plot
    """
    if left_data is None and right_data is None:
        return

    # Determine max number of joints
    left_joints = len(left_data["joint_names"]) if left_data else 0
    right_joints = len(right_data["joint_names"]) if right_data else 0
    max_joints = max(left_joints, right_joints)

    if max_joints == 0:
        return

    # Create subplots - one row per joint pair
    fig, axes = plt.subplots(max_joints, 1, figsize=(14, 2.5 * max_joints), sharex=True)

    if max_joints == 1:
        axes = [axes]

    # Normalize time to start from 0
    left_time = None
    right_time = None
    if left_data:
        left_time = left_data["timestamps"] - left_data["timestamps"][0]
    if right_data:
        right_time = right_data["timestamps"] - right_data["timestamps"][0]

    for i in range(max_joints):
        ax = axes[i]

        left_name = left_data["joint_names"][i] if left_data and i < left_joints else None
        right_name = right_data["joint_names"][i] if right_data and i < right_joints else None

        # Plot left arm (blue)
        if left_data and i < left_joints:
            ax.plot(left_time, left_data["efforts"][:, i], color="steelblue", linewidth=0.8,
                    label=f"Left: {left_name}", alpha=0.8)

        # Plot right arm (red)
        if right_data and i < right_joints:
            ax.plot(right_time, right_data["efforts"][:, i], color="coral", linewidth=0.8,
                    label=f"Right: {right_name}", alpha=0.8)

        # Set y-axis label
        joint_label = ""
        if left_name and right_name:
            joint_label = f"Joint {i+1}"
        elif left_name:
            joint_label = left_name
        elif right_name:
            joint_label = right_name
        ax.set_ylabel(f"{joint_label}\nEffort (Nm)")

        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color="k", linewidth=0.5)
        ax.legend(loc="upper right", fontsize=8)

        # Add statistics text
        stats_text = []
        if left_data and i < left_joints:
            max_val = np.max(left_data["efforts"][:, i])
            mean_val = np.mean(left_data["efforts"][:, i])
            stats_text.append(f"L: max={max_val:.2f}, mean={mean_val:.2f}")
        if right_data and i < right_joints:
            max_val = np.max(right_data["efforts"][:, i])
            mean_val = np.mean(right_data["efforts"][:, i])
            stats_text.append(f"R: max={max_val:.2f}, mean={mean_val:.2f}")

        if stats_text:
            ax.text(
                0.02, 0.95, "\n".join(stats_text),
                transform=ax.transAxes, ha="left", va="top", fontsize=7,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            )

    axes[-1].set_xlabel("Time (s)")

    if title:
        fig.suptitle(title, fontsize=12)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"      Dual arm effort comparison plot saved to: {output_path}")


def plot_effort_summary(all_efforts_data, output_path, title=None):
    """Plot summary of effort statistics across multiple bags.

    Args:
        all_efforts_data: List of dictionaries from calculate_effort_statistics()
        output_path: Path to save the plot
        title: Optional title for the plot
    """
    if not all_efforts_data or len(all_efforts_data) == 0:
        return

    # Get joint names from first valid entry
    joint_names = None
    for data in all_efforts_data:
        if data is not None and "joint_names" in data:
            joint_names = data["joint_names"]
            break

    if joint_names is None:
        return

    num_joints = len(joint_names)

    # Collect statistics for each joint
    max_efforts = []
    min_efforts = []
    mean_efforts = []

    for data in all_efforts_data:
        if data is not None:
            max_efforts.append(data["effort_abs_max"])
            min_efforts.append(data["effort_min"])
            mean_efforts.append(data["effort_mean"])

    if len(max_efforts) == 0:
        return

    max_efforts = np.array(max_efforts)
    min_efforts = np.array(min_efforts)
    mean_efforts = np.array(mean_efforts)

    # Create bar plot
    fig, ax = plt.subplots(figsize=(12, 6))

    x = np.arange(num_joints)
    width = 0.25

    # Plot mean and std
    mean_of_max = np.mean(max_efforts, axis=0)
    std_of_max = np.std(max_efforts, axis=0)
    mean_of_mean = np.mean(mean_efforts, axis=0)
    std_of_mean = np.std(mean_efforts, axis=0)

    bars1 = ax.bar(
        x - width / 2, mean_of_max, width, yerr=std_of_max, label="Max |Effort| (mean±std)", capsize=3, color="coral"
    )
    bars2 = ax.bar(
        x + width / 2,
        np.abs(mean_of_mean),
        width,
        yerr=std_of_mean,
        label="Mean |Effort| (mean±std)",
        capsize=3,
        color="steelblue",
    )

    ax.set_xlabel("Joint")
    ax.set_ylabel("Effort (Nm)")
    ax.set_title(title if title else "Effort Statistics Summary")
    ax.set_xticks(x)
    ax.set_xticklabels(joint_names, rotation=45, ha="right")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Effort summary plot saved to: {output_path}")


def get_bag_completion_time(bag_path):
    """Get completion time (duration) of a bag file."""
    try:
        bag = rosbag.Bag(bag_path, "r")

        # Get start and end time
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()

        bag.close()

        completion_time = end_time - start_time
        return completion_time

    except Exception as e:
        print(f"Error getting completion time from {bag_path}: {e}")
        return None


def process_bag_file(bag_path, penalty_factor=2.0, plot_dir=None):
    """Process a single bag file and return statistics.

    Args:
        bag_path: Path to the bag file
        penalty_factor: Multiplier for completion time if bag name contains 'failed' (default: 2.0)
        plot_dir: Directory to save effort plots (if None, no plots are generated)
    """
    # Fixed topic names for left and right arm
    LEFT_ARM_TOPIC = "/robot/arm_left/joint_states_single"
    RIGHT_ARM_TOPIC = "/robot/arm_right/joint_states_single"

    stats = {
        "completion_time": None,
        "avg_min_distance": None,
        "linear_jerk_rms": None,
        "angular_jerk_rms": None,
        "effort_stats": None,
        "left_arm_data": None,
        "right_arm_data": None,
    }

    # Get completion time
    completion_time = get_bag_completion_time(bag_path)

    # Apply penalty if bag name contains 'failed'
    bag_name = os.path.basename(bag_path).lower()
    if "fail" in bag_name and completion_time is not None:
        completion_time = completion_time * penalty_factor

    stats["completion_time"] = completion_time

    # Get average minimum distance
    distances = read_pointcloud_distances(bag_path)
    if distances is not None and len(distances) > 0:
        stats["avg_min_distance"] = np.mean(distances)

    # Get jerk RMS values
    jerk_stats = calculate_jerk_from_odom(bag_path)
    if jerk_stats is not None:
        stats["linear_jerk_rms"] = jerk_stats["linear_jerk_rms"]
        stats["angular_jerk_rms"] = jerk_stats["angular_jerk_rms"]

    # Get joint states effort data for left and right arms separately
    left_arm_data = read_joint_states_effort(bag_path, topic=LEFT_ARM_TOPIC)
    right_arm_data = read_joint_states_effort(bag_path, topic=RIGHT_ARM_TOPIC)

    # Calculate effort statistics for each arm
    left_effort_stats = calculate_effort_statistics(left_arm_data) if left_arm_data else None
    right_effort_stats = calculate_effort_statistics(right_arm_data) if right_arm_data else None

    # Merge for combined statistics
    combined_effort_stats = merge_arm_effort_statistics(left_effort_stats, right_effort_stats)

    stats["effort_stats"] = {
        "left_arm": left_effort_stats,
        "right_arm": right_effort_stats,
        "combined": combined_effort_stats,
    }
    stats["left_arm_data"] = left_arm_data
    stats["right_arm_data"] = right_arm_data

    # Generate individual bag effort plots if plot_dir is specified
    if plot_dir is not None:
        bag_basename = os.path.splitext(os.path.basename(bag_path))[0]

        # Plot left arm independently
        if left_arm_data is not None:
            plot_path = os.path.join(plot_dir, f"{bag_basename}_effort_left_arm.png")
            plot_effort_curves(left_arm_data, plot_path, title=f"Left Arm Effort - {bag_basename}")

        # Plot right arm independently
        if right_arm_data is not None:
            plot_path = os.path.join(plot_dir, f"{bag_basename}_effort_right_arm.png")
            plot_effort_curves(right_arm_data, plot_path, title=f"Right Arm Effort - {bag_basename}")

        # Plot comparison
        if left_arm_data is not None or right_arm_data is not None:
            plot_path = os.path.join(plot_dir, f"{bag_basename}_effort_comparison.png")
            plot_dual_arm_effort_curves(left_arm_data, right_arm_data, plot_path,
                                        title=f"Left vs Right Arm Effort - {bag_basename}")

    return stats


def process_subfolder(
    subfolder_path, subfolder_name, penalty_factor=2.0, plot_efforts=False
):
    """Process all bag files in a subfolder.

    Args:
        subfolder_path: Path to the subfolder containing bag files
        subfolder_name: Name of the subfolder (method name)
        penalty_factor: Multiplier for completion time if bag name contains 'failed' (default: 2.0)
        plot_efforts: Whether to generate effort plots (default: False)
    """
    # Find all bag files
    bag_files = []
    for file in os.listdir(subfolder_path):
        if file.endswith(".bag"):
            bag_files.append(os.path.join(subfolder_path, file))

    if len(bag_files) == 0:
        print(f"  No bag files found in {subfolder_name}")
        return None

    bag_files.sort()

    print(f"  Processing {len(bag_files)} bag files...")

    # Create plot directory if needed
    plot_dir = None
    if plot_efforts:
        plot_dir = os.path.join(subfolder_path, "effort_plots")
        os.makedirs(plot_dir, exist_ok=True)

    completion_times = []
    avg_min_distances = []
    linear_jerk_rms_list = []
    angular_jerk_rms_list = []
    left_arm_effort_stats_list = []
    right_arm_effort_stats_list = []
    combined_effort_stats_list = []

    for bag_path in tqdm(bag_files, desc=f"    {subfolder_name}", leave=False):
        stats = process_bag_file(
            bag_path, penalty_factor=penalty_factor, plot_dir=plot_dir
        )

        if stats["completion_time"] is not None:
            completion_times.append(stats["completion_time"])

        if stats["avg_min_distance"] is not None:
            avg_min_distances.append(stats["avg_min_distance"])

        if stats["linear_jerk_rms"] is not None:
            linear_jerk_rms_list.append(stats["linear_jerk_rms"])

        if stats["angular_jerk_rms"] is not None:
            angular_jerk_rms_list.append(stats["angular_jerk_rms"])

        if stats["effort_stats"] is not None:
            if stats["effort_stats"]["left_arm"] is not None:
                left_arm_effort_stats_list.append(stats["effort_stats"]["left_arm"])
            if stats["effort_stats"]["right_arm"] is not None:
                right_arm_effort_stats_list.append(stats["effort_stats"]["right_arm"])
            if stats["effort_stats"]["combined"] is not None:
                combined_effort_stats_list.append(stats["effort_stats"]["combined"])

    # Generate summary effort plots
    if plot_efforts:
        if len(left_arm_effort_stats_list) > 0:
            summary_plot_path = os.path.join(subfolder_path, f"{subfolder_name}_effort_summary_left_arm.png")
            plot_effort_summary(left_arm_effort_stats_list, summary_plot_path,
                                title=f"Left Arm Effort Summary - {subfolder_name}")
        if len(right_arm_effort_stats_list) > 0:
            summary_plot_path = os.path.join(subfolder_path, f"{subfolder_name}_effort_summary_right_arm.png")
            plot_effort_summary(right_arm_effort_stats_list, summary_plot_path,
                                title=f"Right Arm Effort Summary - {subfolder_name}")
        if len(combined_effort_stats_list) > 0:
            summary_plot_path = os.path.join(subfolder_path, f"{subfolder_name}_effort_summary_combined.png")
            plot_effort_summary(combined_effort_stats_list, summary_plot_path,
                                title=f"Combined Effort Summary - {subfolder_name}")

    # Calculate averages
    result = {}

    if len(completion_times) > 0:
        result["avg_completion_time"] = np.mean(completion_times)
        result["num_bags_completion"] = len(completion_times)
    else:
        result["avg_completion_time"] = None
        result["num_bags_completion"] = 0

    if len(avg_min_distances) > 0:
        result["avg_min_distance"] = np.mean(avg_min_distances)
        result["num_bags_distance"] = len(avg_min_distances)
    else:
        result["avg_min_distance"] = None
        result["num_bags_distance"] = 0

    if len(linear_jerk_rms_list) > 0:
        result["avg_linear_jerk_rms"] = np.mean(linear_jerk_rms_list)
        result["num_bags_linear_jerk"] = len(linear_jerk_rms_list)
    else:
        result["avg_linear_jerk_rms"] = None
        result["num_bags_linear_jerk"] = 0

    if len(angular_jerk_rms_list) > 0:
        result["avg_angular_jerk_rms"] = np.mean(angular_jerk_rms_list)
        result["num_bags_angular_jerk"] = len(angular_jerk_rms_list)
    else:
        result["avg_angular_jerk_rms"] = None
        result["num_bags_angular_jerk"] = 0

    # Helper function to aggregate effort statistics from a list
    def aggregate_effort_stats(stats_list):
        if len(stats_list) == 0:
            return None

        # Get joint names from first entry
        joint_names = stats_list[0]["joint_names"]

        # Collect per-joint statistics
        all_max = np.array([s["effort_abs_max"] for s in stats_list])
        all_min = np.array([s["effort_min"] for s in stats_list])
        all_mean = np.array([s["effort_mean"] for s in stats_list])
        all_energy_mech = np.array([s["total_energy_mechanical"] for s in stats_list])
        all_energy_torque = np.array([s["total_energy_torque_time"] for s in stats_list])
        all_power_mech = np.array([s["avg_power_mechanical"] for s in stats_list])
        all_power_torque = np.array([s["avg_power_torque_time"] for s in stats_list])

        return {
            "joint_names": joint_names,
            "num_bags_effort": len(stats_list),
            # Per-joint statistics (averaged across bags)
            "effort_max_per_joint": np.mean(all_max, axis=0).tolist(),
            "effort_min_per_joint": np.mean(all_min, axis=0).tolist(),
            "effort_mean_per_joint": np.mean(all_mean, axis=0).tolist(),
            "effort_max_per_joint_std": np.std(all_max, axis=0).tolist(),
            # Global statistics (across all joints and bags)
            "global_effort_max": float(np.max(all_max)),
            "global_effort_min": float(np.min(all_min)),
            # Energy statistics
            "avg_total_energy_mechanical": float(np.mean(all_energy_mech)),
            "std_total_energy_mechanical": float(np.std(all_energy_mech)),
            "avg_total_energy_torque_time": float(np.mean(all_energy_torque)),
            "std_total_energy_torque_time": float(np.std(all_energy_torque)),
            # Power statistics
            "avg_power_mechanical": float(np.mean(all_power_mech)),
            "std_power_mechanical": float(np.std(all_power_mech)),
            "avg_power_torque_time": float(np.mean(all_power_torque)),
            "std_power_torque_time": float(np.std(all_power_torque)),
        }

    # Aggregate effort statistics for left arm, right arm, and combined
    result["effort"] = {
        "left_arm": aggregate_effort_stats(left_arm_effort_stats_list),
        "right_arm": aggregate_effort_stats(right_arm_effort_stats_list),
        "combined": aggregate_effort_stats(combined_effort_stats_list),
    }

    return result


def convert_to_json_serializable(obj):
    """Convert numpy types and other non-serializable types to JSON-compatible types."""
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: convert_to_json_serializable(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_json_serializable(item) for item in obj]
    elif obj is None:
        return None
    else:
        return obj


def main():
    parser = argparse.ArgumentParser(description="Analyze rosbag statistics for a task")
    parser.add_argument(
        "--task_name", type=str, default="TriPilot-FF-T2-GateTurn", help="Task name (dataset folder name)"
    )
    parser.add_argument(
        "--dataset_dir",
        type=str,
        default="/home/jeong/zeno/wholebody-teleop/act/dataset",
        help="Base directory for datasets (default: dataset)",
    )
    parser.add_argument(
        "--output_json",
        type=str,
        default=None,
        help="Output JSON file path (default: <task_name>_stats.json in dataset directory)",
    )
    parser.add_argument(
        "--penalty_factor",
        type=float,
        default=2.5,
        help='Penalty multiplier for completion time if bag name contains "failed" (default: 2.5)',
    )
    parser.add_argument(
        "--plot_efforts", action="store_true", help="Generate effort plots for each bag file and summary plots"
    )

    args = parser.parse_args()

    # Find dataset folder
    dataset_path = os.path.join(args.dataset_dir, args.task_name)

    if not os.path.exists(dataset_path):
        print(f"Error: Dataset folder not found: {dataset_path}")
        sys.exit(1)

    if not os.path.isdir(dataset_path):
        print(f"Error: {dataset_path} is not a directory")
        sys.exit(1)

    print(f"Analyzing dataset: {dataset_path}")
    print("=" * 80)

    # Find subfolders (methods)
    subfolders = []
    for item in os.listdir(dataset_path):
        item_path = os.path.join(dataset_path, item)
        if os.path.isdir(item_path):
            # Check if subfolder contains bag files
            has_bags = any(f.endswith(".bag") for f in os.listdir(item_path))
            if has_bags:
                subfolders.append((item, item_path))

    # Also check if there are bag files directly in the dataset folder
    has_bags_root = any(f.endswith(".bag") for f in os.listdir(dataset_path))

    if len(subfolders) == 0 and not has_bags_root:
        print(f"Error: No bag files found in {dataset_path} or its subfolders")
        sys.exit(1)

    results = {}

    # Process subfolders
    for subfolder_name, subfolder_path in subfolders:
        print(f"\nProcessing method: {subfolder_name}")
        result = process_subfolder(
            subfolder_path,
            subfolder_name,
            penalty_factor=args.penalty_factor,
            plot_efforts=args.plot_efforts,
        )
        if result is not None:
            results[subfolder_name] = result

    # Process root folder if it has bags
    if has_bags_root:
        print(f"\nProcessing root folder")
        result = process_subfolder(
            dataset_path,
            "root",
            penalty_factor=args.penalty_factor,
            plot_efforts=args.plot_efforts,
        )
        if result is not None:
            results["root"] = result

    # Print summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    if len(results) == 0:
        print("No valid results found")
        return

    # Print table header
    print(
        f"\n{'Method':<20} {'Avg Completion Time (s)':<25} {'Avg Min Distance (m)':<25} {'Linear Jerk RMS':<20} {'Angular Jerk RMS':<20}"
    )
    print("-" * 110)

    for method_name, result in sorted(results.items()):
        completion_str = f"{result['avg_completion_time']:.2f}" if result["avg_completion_time"] is not None else "N/A"
        distance_str = f"{result['avg_min_distance']:.4f}" if result["avg_min_distance"] is not None else "N/A"
        linear_jerk_str = f"{result['avg_linear_jerk_rms']:.6f}" if result["avg_linear_jerk_rms"] is not None else "N/A"
        angular_jerk_str = (
            f"{result['avg_angular_jerk_rms']:.6f}" if result["avg_angular_jerk_rms"] is not None else "N/A"
        )

        print(f"{method_name:<20} {completion_str:<25} {distance_str:<25} {linear_jerk_str:<20} {angular_jerk_str:<20}")

    # Print effort statistics summary table (for combined/total)
    print("\n" + "-" * 130)
    print(
        f"{'Method':<20} {'Arm':<10} {'Avg Energy (Mech)':<20} {'Avg Energy (Torque)':<20} {'Avg Power (Mech)':<20} {'Global Max Effort':<20}"
    )
    print("-" * 130)

    for method_name, result in sorted(results.items()):
        effort_data = result.get("effort")
        if effort_data is not None:
            for arm_label, arm_key in [("Left", "left_arm"), ("Right", "right_arm"), ("Combined", "combined")]:
                effort = effort_data.get(arm_key)
                if effort is not None:
                    energy_mech_str = f"{effort['avg_total_energy_mechanical']:.2f}"
                    energy_torque_str = f"{effort['avg_total_energy_torque_time']:.2f}"
                    power_mech_str = f"{effort['avg_power_mechanical']:.2f}"
                    max_effort_str = f"{effort['global_effort_max']:.2f}"
                    print(
                        f"{method_name:<20} {arm_label:<10} {energy_mech_str:<20} {energy_torque_str:<20} {power_mech_str:<20} {max_effort_str:<20}"
                    )
                else:
                    print(f"{method_name:<20} {arm_label:<10} {'N/A':<20} {'N/A':<20} {'N/A':<20} {'N/A':<20}")
        else:
            print(f"{method_name:<20} {'N/A':<10} {'N/A':<20} {'N/A':<20} {'N/A':<20} {'N/A':<20}")

    # Print detailed statistics
    print("\n" + "=" * 80)
    print("DETAILED STATISTICS")
    print("=" * 80)

    for method_name, result in sorted(results.items()):
        print(f"\n{method_name}:")
        print(f"  Completion Time:")
        print(
            f"    Average: {result['avg_completion_time']:.2f} s"
            if result["avg_completion_time"] is not None
            else "    Average: N/A"
        )
        print(f"    Number of bags: {result['num_bags_completion']}")
        print(f"  Minimum Distance:")
        print(
            f"    Average: {result['avg_min_distance']:.4f} m"
            if result["avg_min_distance"] is not None
            else "    Average: N/A"
        )
        print(f"    Number of bags: {result['num_bags_distance']}")
        print(f"  Linear Jerk RMS:")
        print(
            f"    Average: {result['avg_linear_jerk_rms']:.6f}"
            if result["avg_linear_jerk_rms"] is not None
            else "    Average: N/A"
        )
        print(f"    Number of bags: {result['num_bags_linear_jerk']}")
        print(f"  Angular Jerk RMS:")
        print(
            f"    Average: {result['avg_angular_jerk_rms']:.6f}"
            if result["avg_angular_jerk_rms"] is not None
            else "    Average: N/A"
        )
        print(f"    Number of bags: {result['num_bags_angular_jerk']}")

        # Print effort statistics for each arm
        effort_data = result.get("effort")
        if effort_data is not None:
            for arm_label, arm_key in [("Left Arm", "left_arm"), ("Right Arm", "right_arm"), ("Combined (Both Arms)", "combined")]:
                effort = effort_data.get(arm_key)
                if effort is not None:
                    print(f"  {arm_label} Effort Statistics:")
                    print(f"    Number of bags: {effort['num_bags_effort']}")
                    print(f"    Global Max Effort: {effort['global_effort_max']:.4f} Nm")
                    print(f"    Global Min Effort: {effort['global_effort_min']:.4f} Nm")
                    print(f"    Energy (Mechanical):")
                    print(f"      Average: {effort['avg_total_energy_mechanical']:.4f} J")
                    print(f"      Std: {effort['std_total_energy_mechanical']:.4f} J")
                    print(f"    Energy (Torque-Time, proportional to current):")
                    print(f"      Average: {effort['avg_total_energy_torque_time']:.4f} Nm*s")
                    print(f"      Std: {effort['std_total_energy_torque_time']:.4f} Nm*s")
                    print(f"    Power (Mechanical):")
                    print(f"      Average: {effort['avg_power_mechanical']:.4f} W")
                    print(f"      Std: {effort['std_power_mechanical']:.4f} W")
                    print(f"    Power (Torque-Time):")
                    print(f"      Average: {effort['avg_power_torque_time']:.4f} Nm/s")
                    print(f"      Std: {effort['std_power_torque_time']:.4f} Nm/s")
                    print(f"    Per-Joint Max |Effort| (Nm):")
                    for i, name in enumerate(effort["joint_names"]):
                        print(
                            f"      {name}: {effort['effort_max_per_joint'][i]:.4f} +/- {effort['effort_max_per_joint_std'][i]:.4f}"
                        )
                else:
                    print(f"  {arm_label} Effort Statistics: N/A")
        else:
            print(f"  Effort Statistics: N/A")

    # Save results to JSON file
    if args.output_json is None:
        # Default output path: <dataset_dir>/<task_name>_stats.json
        output_json_path = os.path.join(args.dataset_dir, f"{args.task_name}_stats.json")
    else:
        output_json_path = args.output_json

    # Prepare JSON output
    json_output = {
        "task_name": args.task_name,
        "dataset_path": dataset_path,
        "results": convert_to_json_serializable(results),
    }

    # Write JSON file
    try:
        with open(output_json_path, "w", encoding="utf-8") as f:
            json.dump(json_output, f, indent=2, ensure_ascii=False)
        print(f"\n" + "=" * 80)
        print(f"Results saved to: {output_json_path}")
    except Exception as e:
        print(f"\nWarning: Failed to save JSON file: {e}")


if __name__ == "__main__":
    main()

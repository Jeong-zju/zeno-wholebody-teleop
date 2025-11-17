
//
// Created by qiayuan on 6/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics.
********************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>

// ROS control

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <dm_common/HybridJointInterface.h>
#include "dm_hw/damiao.h"

// 这主要是硬件通信传输数据
namespace damiao
{
	class DmHW : public hardware_interface::RobotHW
	{
	public:
		DmHW() = default;

		bool parseDmActData(XmlRpc::XmlRpcValue &act_datas, ros::NodeHandle &robot_hw_nh);
		/** \brief Get necessary params from param server. Init hardware_interface.
		 *
		 * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
		 * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
		 *
		 * @param root_nh Root node-handle of a ROS node.
		 * @param robot_hw_nh Node-handle for robot hardware.
		 * @return True when init successful, False when failed.
		 */
		bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
		void read(const ros::Time &time, const ros::Duration &period, ros::Publisher pub);
		void write(const ros::Time &time, const ros::Duration &period) override;

	protected:
		// 一个serial一个元素
		std::vector<std::shared_ptr<Motor_Control>> motor_ports_{};
		// 通过名称、can_id找到对应的电机
		std::unordered_map<std::string, std::unordered_map<int, DmActData>> port_id2dm_data_{};

		// Interface
		hardware_interface::JointStateInterface jointStateInterface_; // NOLINT(misc-non-private-member-variables-in-classes)
		HybridJointInterface hybridJointInterface_;					  // NOLINT(misc-non-private-member-variables-in-classes)

	private:
	};

} // namespace legged


//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics.
********************************************************************************/

#include "dm_hw/DmHW.h"

#include <xmlrpcpp/XmlRpcException.h>
namespace damiao
{

	bool DmHW::parseDmActData(XmlRpc::XmlRpcValue &act_datas, ros::NodeHandle &robot_hw_nh)
	{
		ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
		try
		{
			for (auto it = act_datas.begin(); it != act_datas.end(); ++it)
			{
				if (!it->second.hasMember("port"))
				{
					ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated port.");
					continue;
				}
				else if (!it->second.hasMember("type"))
				{
					ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated type.");
					continue;
				}
				else if (!it->second.hasMember("can_id"))
				{
					ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated CAN_ID.");
					continue;
				}
				else if (!it->second.hasMember("mst_id"))
				{
					ROS_ERROR_STREAM("DmActuator " << it->first << " has no associated MST_ID.");
					continue;
				}

				std::string port = act_datas[it->first]["port"];
				std::string type = act_datas[it->first]["type"];
				int can_id = static_cast<int>(act_datas[it->first]["can_id"]);
				int mst_id = static_cast<int>(act_datas[it->first]["mst_id"]);
				ROS_ERROR_STREAM("DmActuator " << it->first << " port: " << port << " type: " << type << " can_id: " << can_id << " mst_id: " << mst_id);
				DM_Motor_Type motorType;

				if (type == "DM4310")
					motorType = DM4310;
				else if (type == "DM4310_48V")
					motorType = DM4310_48V;
				else if (type == "DM4340")
					motorType = DM4340;
				else if (type == "DM4340_48V")
					motorType = DM4340_48V;
				else if (type == "DM6006")
					motorType = DM6006;
				else if (type == "DM8006")
					motorType = DM8006;
				else if (type == "DM8009")
					motorType = DM8009;
				else if (type == "DM10010L")
					motorType = DM10010L;
				else if (type == "DM10010")
					motorType = DM10010;
				else if (type == "DMH3510")
					motorType = DMH3510;
				else if (type == "DMH6215")
					motorType = DMH6215;
				else if (type == "DMG6220")
					motorType = DMG6220;
				else
				{
					ROS_ERROR("Unknown motor type specified: %s", type.c_str());
				}
				// for bus interface
				if (port_id2dm_data_.find(port) == port_id2dm_data_.end())
					port_id2dm_data_.insert(std::make_pair(port, std::unordered_map<int, DmActData>()));

				if (!(port_id2dm_data_[port].find(can_id) == port_id2dm_data_[port].end()))
				{
					ROS_ERROR_STREAM("Repeat dmactuator on bus " << port << " and ID " << can_id);
					return false;
				}
				else
				{
					port_id2dm_data_[port].insert(std::make_pair(can_id, DmActData{.name = it->first,
																				   .motorType = motorType,
																				   .can_id = can_id,
																				   .mst_id = mst_id,
																				   .pos = 0,
																				   .vel = 0,
																				   .effort = 0,
																				   .cmd_pos = 0,
																				   .cmd_vel = 0,
																				   .cmd_effort = 0}));
				}
				// for ros_control interface
				hardware_interface::JointStateHandle state_handle(port_id2dm_data_[port][can_id].name, &port_id2dm_data_[port][can_id].pos,
																  &port_id2dm_data_[port][can_id].vel,
																  &port_id2dm_data_[port][can_id].effort);
				jointStateInterface_.registerHandle(state_handle);
				hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &port_id2dm_data_[port][can_id].cmd_pos,
																	   &port_id2dm_data_[port][can_id].cmd_vel, &port_id2dm_data_[port][can_id].kp,
																	   &port_id2dm_data_[port][can_id].kd, &port_id2dm_data_[port][can_id].cmd_effort));
			}
		}
		catch (XmlRpc::XmlRpcException &e)
		{
			ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
							 << "configuration: " << e.getMessage() << ".\n"
							 << "Please check the dmconfiguration, particularly parameter types.");
			return false;
		}
		return true;
	}

	bool DmHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
	{
		XmlRpc::XmlRpcValue xml_rpc_value;

		registerInterface(&jointStateInterface_);  // 注册jointStateInterface_接口
		registerInterface(&hybridJointInterface_); // 注册hybridJointInterface_接口

		if (!robot_hw_nh.getParam("dm_actuators", xml_rpc_value))
			ROS_WARN("No dm_actuators specified");
		else if (!parseDmActData(xml_rpc_value, robot_hw_nh))
			return false;

		if (!robot_hw_nh.getParam("serials", xml_rpc_value))
			ROS_WARN("No serials specified");
		else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
			{
				if (!it->second.hasMember("port"))
				{
					ROS_ERROR_STREAM("Serials " << it->first << " has no associated port.");
					continue;
				}
				else if (!it->second.hasMember("baudrate"))
				{
					ROS_ERROR_STREAM("Serials " << it->first << " has no associated baudrate.");
					continue;
				}
				std::string port = xml_rpc_value[it->first]["port"];
				int baudrate = static_cast<int>(xml_rpc_value[it->first]["baudrate"]);
				std::cerr << "Serials " << it->first << " port: " << port << " baudrate: " << baudrate << std::endl;

				motor_ports_.push_back(std::make_shared<Motor_Control>(port, baudrate, &port_id2dm_data_[port]));
			}
		}

		return true;
	}

	void DmHW::read(const ros::Time &time, const ros::Duration &period, ros::Publisher pub)
	{
		for (auto motor_port : motor_ports_)
		{
			motor_port->read(pub);
		}
	}

	void DmHW::write(const ros::Time &time, const ros::Duration &period)
	{
		for (auto motor_port : motor_ports_)
		{
			motor_port->write();
		}
	}

} // namespace damiao

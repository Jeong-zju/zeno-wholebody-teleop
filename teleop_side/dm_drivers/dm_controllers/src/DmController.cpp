#include "dm_controllers/DmController.h"

namespace damiao
{

	bool DmController::init(HybridJointInterface *robot_hw, ros::NodeHandle &nh)
	{
		std::cerr << "Successfully got HybridEffort joint interface" << std::endl;

		std::vector<std::string> joint_names{"joint0_motor", "joint1_motor", "joint2_motor"};
		// std::vector<std::string> joint_names{"joint0_motor", "joint1_motor"};
		for (const auto &joint_name : joint_names)
		{
			hybridJointHandles_.push_back(robot_hw->getHandle(joint_name));
		}

		return true;
	}

	void DmController::starting(const ros::Time &time)
	{
		// ROS_INFO("DmController started.");
		std::cerr << "DmController started." << std::endl;
	}

	void DmController::update(const ros::Time &time, const ros::Duration &period)
	{
		// 设置关节命令
		// std::cerr<<"dmcontroller update"<<std::endl;
		// std::cerr<<"size: "<<hybridJointHandles_.size()<<std::endl;
		float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
		// hybridJointHandles_[0].setCommand(0.0, q*-3.0,0.0,0.3,0.0);
		hybridJointHandles_[0].setCommand(0.0, 0.0, 5.0, 0.1, 0.0);
		hybridJointHandles_[1].setCommand(0.0, 0.0, 5.0, 0.1, 0.0);
		hybridJointHandles_[2].setCommand(0.0, 0.0, 5.0, 0.1, 0.0);
		// hybridJointHandles_[1].setCommand(0.0, q*2.0,0.0,0.3,0.0);
		// hybridJointHandles_[2].setCommand(0.0, q*-5.0,0.0,0.3,0.0);
	}

	void DmController::stopping(const ros::Time &time)
	{
		// ROS_INFO("DmController stopped.");
		std::cerr << "DmController stop." << std::endl;
	}

} // namespace damiao

// 注册插件
PLUGINLIB_EXPORT_CLASS(damiao::DmController, controller_interface::ControllerBase);
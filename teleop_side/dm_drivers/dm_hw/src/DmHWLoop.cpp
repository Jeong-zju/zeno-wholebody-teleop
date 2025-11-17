
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics.
********************************************************************************/

#include "dm_hw/DmHWLoop.h"
#include <geometry_msgs/Twist.h>

namespace damiao
{
	DmHWLoop::DmHWLoop(ros::NodeHandle &nh, std::shared_ptr<DmHW> hardware_interface)
		: nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true)
	{
		controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

		int error = 0;
		int threadPriority = 0;
		ros::NodeHandle nhP("damiao");
		error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
		error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
		error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
		if (error > 0)
		{
			std::string error_message =
				"could not retrieve one of the required parameters: damiao/loop_hz or damiao/cycle_time_error_threshold or damiao/thread_priority";

			ROS_ERROR_STREAM(error_message);
			throw std::runtime_error(error_message);
		}

		lastTime_ = Clock::now();

		this->cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 1);

		loopThread_ = std::thread([&]()
								  {
    while (loopRunning_)
    {
		// std::cerr << "DmHWLoop::update()" << std::endl;
      update();
    } });
		sched_param sched{.sched_priority = threadPriority};
		if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0)
		{
			ROS_WARN(
				"Failed to set threads priority (one possible reason could be that the user and the group permissions "
				"are not set properly.).\n");
		}
	}

	void DmHWLoop::update()
	{
		const auto currentTime = Clock::now();

		const Duration desiredDuration(1.0 / loopHz_);

		Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
		elapsedTime_ = ros::Duration(time_span.count());
		lastTime_ = currentTime;

		const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
		if (cycle_time_error > cycleTimeErrorThreshold_)
		{
			ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
																	   << "cycle time: " << elapsedTime_ << "s, "
																	   << "threshold: " << cycleTimeErrorThreshold_ << "s");
		}

		// std::cerr << "fuck" << std::endl;

		hardwareInterface_->read(ros::Time::now(), elapsedTime_, this->cmd_pub_);

		controllerManager_->update(ros::Time::now(), elapsedTime_);

		hardwareInterface_->write(ros::Time::now(), elapsedTime_);

		const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
		std::this_thread::sleep_until(sleepTill);
	}

	DmHWLoop::~DmHWLoop()
	{
		loopRunning_ = false;
		if (loopThread_.joinable())
		{
			loopThread_.join();
		}
	}

} // namespace damiao

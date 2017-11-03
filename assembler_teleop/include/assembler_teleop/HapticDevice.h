#ifndef HAPTIC_DEVICE_H__
#define HAPTIC_DEVICE_H__

// This c++ project follow google name rules: https://google.github.io/styleguide/cppguide.html#General_Naming_Rules
// If you follow these rules, you will get all the meaning of the names.

#include <mutex>
#include <thread>
#include <vector>

#include "ros/ros.h"

#include "std_msgs/Int8MultiArray.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>


class HapticDevice
{
	public:
		HapticDevice();
		virtual ~HapticDevice();

		bool Connect();
		bool IsConnected();
		void EnableDevice();
		void DisableDevice();
		bool IsLocked();
		void SetLocked(bool locked);
		void SetDeviceID(int dev_id);
		int GetDeviceID();

		// core API for data exchange
		std::vector<double> GetEndPosition();
		std::vector<double> GetFeedbackForce();
		void SetTargetForce(std::vector<double> force,double limit);
		std::vector<double> GetStretch();
		void SetStretch(std::vector<double> stretch);
		void PositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr &position_data);
		void ButtonStateCallback(const std_msgs::Int8MultiArray::ConstPtr & button_data);

    protected:
		ros::NodeHandle nh_;
		// thread to get data from device
		// void DeviceDataGetThreadRun();
		// std::shared_ptr<std::thread> data_get_thread_;
		ros::Subscriber pos_sub_;
		ros::Subscriber button_state_sub_;
		ros::Publisher set_force_pub_;
		
	private:
		// store current haptic device data
		bool keepalive_;
		bool haptic_is_locked_ = true;  // haptic device lock state
		std::mutex haptic_input_lock_;  // for haptic_is_locked flag sync to avoid modifying haptic_is_locked
		std::mutex haptic_dev_data_lock_;  // for handler pose data sync

		int device_id_;
		int device_count_;
		std::vector<double> end_position_;
		std::vector<double> feedback_force_;
		std::vector<double> stretch_to_robot_;
};

#endif

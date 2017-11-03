#ifndef TELEOP_ROBOT_H__
#define TELEOP_ROBOT_H__

// This c++ project follow google name rules: https://google.github.io/styleguide/cppguide.html#General_Naming_Rules
// If you follow these rules, you will get all the meaning of the names.

#include <mutex>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "assembler_teleop/HapticDevice.h"

namespace Teleop {
	enum OperateMode {TELEOP_MODE, AUTO_MODE, SHARED_AUTONOMY_MODE};
}

typedef Teleop::OperateMode RobotOperateMode;

class Communication 
{
	public:
		Communication();
		virtual ~Communication();
};

class TeleopRobot
{
public:
	TeleopRobot();
	virtual ~TeleopRobot();

	bool ConnectHapticDevice();

	// enable/disable robot
	void EnableRobot();
	void DisableRobot();

	// set joint names
	void SetArmJointsName(std::vector<std::string> arm_joints_name);
	void SetToolJointsName(std::vector<std::string> tool_joints_name);
	void SetHapticDevice(std::shared_ptr<HapticDevice> haptic_device);
	std::shared_ptr<HapticDevice> GetHapticDeviceHandler();
	
	// callback member function
	virtual	void RegisterCallback() {};
	virtual	void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){};
	virtual	void ToolWrechCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg){};
	// virtual	void ToolPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& tool_msg);
	// virtual	void ToolVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);  

    // operate
	virtual	void SendCommandTimer(const ros::TimerEvent& event){};
	virtual void SendPoseCmdToRobot(double []){};
	virtual void SendVelocityCmdToRobot(double []){};
	// virtual	void DoAutonomyOP();

	// cartesian space move
	// virtual	int MoveEndSpeedL(double end_relative_pose[]);
	// virtual	int MoveEndL(double pose[], double a=1.2, double v=0.25, double t=0.02, double r=0);
	// virtual	int MoveEndRelativeL(double pose[], double a=1.2, double v=0.25, double t=0.02, double r=0);
	// virtual	int SetToolPose(double tool_pose[], int length);

	// joint space move
	// virtual	int MoveServoj(double pose[], double a, double v, double t=0);

	// get robot states
	// virtual	int GetCurrentJointStates(double angle[], int length);
	// virtual	int GetCurrentWrenchState(double wrech[], int length);
	// virtual	int GetToolPose(double tool_pose[], int length);

	// virtual bool IsAlive();

	// stop
	virtual void start() {};
	// virtual	int halt(); // join the listenHandlerThread

	// lock
	// virtual	void setHandlerLockFlag(bool flag);
	// virtual	bool isHandlerLocked();

protected:
	void SetOperateMode(RobotOperateMode mode);
		// set joint value
	void SetJointStatesValue(std::map<std::string, double> joints_position, std::map<std::string, double> joints_velocity, 
							std::map<std::string, double> joints_effort);

	ros::NodeHandle nh_;
	ros::Subscriber sub_joint_state_;
	ros::Subscriber sub_tool_vel_;
	ros::Subscriber sub_tool_wrench_;
	ros::Subscriber sub_tool_pose_;

	ros::Timer send_command_timer_;

	bool keepalive_ = true;
	RobotOperateMode operate_mode_;
	std::vector<std::string> arm_joints_name_;
	std::vector<std::string> tool_joints_name_;
	std::shared_ptr<HapticDevice> haptic_dev_;
	std::vector<double> haptic_unlock_data_;
	double command_send_period_;
	double control_executation_period_;

	// store joint states
	std::map<std::string, double> joints_position_;
	std::map<std::string, double> joints_velocity_;
	std::map<std::string, double> joints_effort_;

	// store the current robot pose state
	std::vector<double> end_pose_;
	std::vector<double> end_vel_;
	std::vector<double> end_force_;
};


#endif
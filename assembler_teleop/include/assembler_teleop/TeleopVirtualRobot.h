#ifndef TELEOP_VIRTUAL_ROBOT_H__
#define TELEOP_VIRTUAL_ROBOT_H__

#include "assembler_teleop/TeleopRobot.h"
#include <thread>
#include <mutex>

#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include "control_msgs/FollowJointTrajectoryAction.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


class TeleopVirtualRobot: public TeleopRobot
{
	public:
		TeleopVirtualRobot(ros::NodeHandle & nh, RobotOperateMode op_mode);
		virtual ~TeleopVirtualRobot();

		 void RegisterCallback();
		 void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg);
		 void ToolWrechCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg);
		 geometry_msgs::PoseStamped GetCurrentArmEndPose();
		 void SendCommandTimer(const ros::TimerEvent& event);
		 void SendPoseCmdToRobot(Eigen::Affine3d end_pose_cmd);
		//  std::map<std::string, double> GetArmIK(Eigen::Affine3d end_pose);
		 bool GetArmIK(Eigen::Affine3d end_pose, std::map<std::string, double> & ret);
		 void SendJointTrajectoryCmd(std::map<std::string, double>, std::map<std::string, double>, std::map<std::string, double>);
		 void SendSafetyCommand();
		std::vector<double> GetArmEndPose();
		std::map<std::string, double> GetIKSolution();
		void SendJointToRobot(); // we need use joint state including velocity, accelebration as input
		void start();

	protected:
		void ListenInputThreadRun();
		std::shared_ptr<std::thread> listen_input_thread_;
		sensor_msgs::JointState joint_states_msg_;
		moveit::planning_interface::MoveGroupInterface arm_commander_;
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;

		robot_model_loader::RobotModelLoader robot_model_loader_;
		robot_model::RobotModelPtr kinematic_model_;
		robot_state::RobotStatePtr kinematic_state_;
		robot_state::JointModelGroup *joint_model_group_;

		std::mutex val_lock_;  // for handler position data sync
		std::mutex goal_lock_;
		bool reached_goal_;


};

#endif
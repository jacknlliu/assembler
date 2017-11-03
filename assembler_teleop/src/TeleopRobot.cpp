#include "assembler_teleop/TeleopRobot.h"

#include <math.h>

#include "assembler_teleop/basic_math.h"
#include "assembler_teleop/general_keyboard.h"


TeleopRobot::TeleopRobot()
{
}

TeleopRobot::~TeleopRobot()
{
}

void TeleopRobot::SetOperateMode(RobotOperateMode mode)
{
    operate_mode_ = mode;
}


void TeleopRobot::SetArmJointsName(std::vector<std::string> arm_joints_name)
{
    arm_joints_name_ = arm_joints_name;
    ROS_INFO("arm_joints_name_ size is: %d", arm_joints_name_.size());
}


void TeleopRobot::SetToolJointsName(std::vector<std::string> tool_joints_name)
{
    tool_joints_name_ = tool_joints_name;
    ROS_INFO("arm_joints_name_ size is: %d", tool_joints_name_.size());
}

bool TeleopRobot::ConnectHapticDevice()
{
    std::shared_ptr<HapticDevice> haptic_device = std::make_shared<HapticDevice>();

    // TODO: haptic_device get exactly!
    if (haptic_device)
    {
        if (haptic_device->IsConnected())
        {
            haptic_dev_ = haptic_device;
            ROS_INFO("really connected haptic device?");
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

void TeleopRobot::SetHapticDevice(std::shared_ptr<HapticDevice> haptic_device)
{
    haptic_dev_ = haptic_device;
}


std::shared_ptr<HapticDevice> TeleopRobot::GetHapticDeviceHandler()
{
    return haptic_dev_;
}

void TeleopRobot::EnableRobot()
{
    keepalive_ = true;
}

void TeleopRobot::DisableRobot()
{
    keepalive_ = false;
}


void TeleopRobot::SetJointStatesValue(std::map<std::string, double> joints_position, std::map<std::string, double> joints_velocity, 
                         std::map<std::string, double> joints_effort)
{
    joints_position_ = joints_position;
    joints_velocity_ = joints_velocity;
    joints_effort_ = joints_effort;
}

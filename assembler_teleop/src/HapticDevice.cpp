#include "assembler_teleop/HapticDevice.h"

#include "ros/ros.h"

#include "assembler_teleop/basic_math.h"

HapticDevice::HapticDevice()
{

  if (!Connect())
  {
      device_id_ = -1;
      return;
  }

  haptic_is_locked_ = true;

  feedback_force_.resize(3);
  for (size_t i = 0; i < feedback_force_.size(); i++)
  {
      feedback_force_[i] = 0.0;
  }
  
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  end_position_.resize(3);
  end_position_.push_back(a);
  end_position_.push_back(b);
  end_position_.push_back(c);

  stretch_to_robot_.resize(3);
  std::vector<double> stretch = {0.1, 0.1, 0.1}; // set stretch
  SetStretch(stretch);

  EnableDevice();
}


HapticDevice::~HapticDevice()
{
    DisableDevice();

    // finalize all data memeber
    device_count_ = 0;
    device_id_ = -1;
}


bool HapticDevice::Connect()
{
    pos_sub_ = nh_.subscribe("/haptic/position", 1000, &HapticDevice::PositionCallback, this);
    button_state_sub_ = nh_.subscribe("/haptic/button_state", 1000, &HapticDevice::ButtonStateCallback, this);
    set_force_pub_ = nh_.advertise<geometry_msgs::Vector3>("/haptic/force",1);
    
    return true;
}


void HapticDevice::ButtonStateCallback(const std_msgs::Int8MultiArray::ConstPtr & button_data)
{
}

void HapticDevice::PositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr &position_data)
{
    haptic_dev_data_lock_.lock();

    end_position_[0] = position_data->vector.x;
    end_position_[1] = position_data->vector.y;
    end_position_[2] = position_data->vector.z;

    haptic_dev_data_lock_.unlock();
}


bool HapticDevice::IsConnected()
{
    return true;
}


void HapticDevice::EnableDevice()
{
    keepalive_ = true;
}

void HapticDevice::DisableDevice()
{
    keepalive_ = false;
}

bool HapticDevice::IsLocked()
{
    return haptic_is_locked_;    
}

void HapticDevice::SetLocked(bool locked)
{
    haptic_input_lock_.lock();
    haptic_is_locked_ = locked;
    haptic_input_lock_.unlock();
}

void HapticDevice::SetDeviceID(int dev_id)
{
    device_id_ = dev_id;
}

int HapticDevice::GetDeviceID()
{
    return device_id_;
}

std::vector<double> HapticDevice::GetEndPosition()
{
    return end_position_; 
}

std::vector<double> HapticDevice::GetStretch()
{
    return stretch_to_robot_;
}


void HapticDevice::SetStretch(std::vector<double> stretch)
{

    if (stretch.size() != stretch_to_robot_.size())
    {
        return;
    }
    

    for (size_t i = 0; i < stretch_to_robot_.size(); i++)
    {
        stretch_to_robot_[i] = stretch[i];
    }
}

void HapticDevice::SetTargetForce(std::vector<double> force, double limit)
{
    // double set_force[3] = {0.0, 0.0, 0.0};
    geometry_msgs::Vector3 set_force;

    if (fabs(force.at(0)) >= limit) {
        set_force.x = sign_of_float(force.at(0))*limit;
    }

    if (fabs(force.at(1)) >= limit) {
        set_force.y = sign_of_float(force.at(1))*limit;
    }

    if (fabs(force.at(2)) >= limit) {
        set_force.z = sign_of_float(force.at(2))*limit;
    }

    // set force feedback
    set_force_pub_.publish(set_force);
}


std::vector<double> HapticDevice::GetFeedbackForce()
{

}
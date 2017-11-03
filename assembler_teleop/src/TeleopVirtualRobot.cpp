#include "assembler_teleop/TeleopVirtualRobot.h"
#include "assembler_teleop/general_keyboard.h"


// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// action lib
#include <actionlib/client/simple_action_client.h>
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/terminal_state.h>
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <Eigen/Geometry>

TeleopVirtualRobot::TeleopVirtualRobot(ros::NodeHandle & nh, RobotOperateMode op_mode): ac_("/arm_gazebo_controller/follow_joint_trajectory", true)
,arm_commander_("manipulator"), robot_model_loader_("robot_description")
{
     ac_.waitForServer();

     reached_goal_ = true;

    SetOperateMode(op_mode);
    // init data members
    command_send_period_ = 0.001; // unit: s
    control_executation_period_ = 0.008;

    // better to use moveit API get the joints names or use parse robot_description with group name
    std::vector<std::string> arm_joints_name;
    std::vector<std::string> tool_joints_name;

    nh_ = nh;
//    nh_.getParam("robot_joints/arm_joints", arm_joints_name);
//    nh_.getParam("robot_joints/tool_joints", tool_joints_name);

    arm_joints_name.push_back("shoulder_pan_joint");
    arm_joints_name.push_back("shoulder_lift_joint");
    arm_joints_name.push_back("elbow_joint");
    arm_joints_name.push_back("wrist_1_joint");
    arm_joints_name.push_back("wrist_2_joint");
    arm_joints_name.push_back("wrist_3_joint");

    tool_joints_name.push_back("bh_j11_joint");
    tool_joints_name.push_back("bh_j12_joint");
    tool_joints_name.push_back("bh_j22_joint");
    tool_joints_name.push_back("bh_j32_joint");

// !!! moviet not provide empty parameter construct function, so we must initial our member in initial list firstly.
//    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
//    arm_commander_ = move_group;

    SetArmJointsName(arm_joints_name);
    SetToolJointsName(tool_joints_name);

    kinematic_model_ = robot_model_loader_.getModel();
    kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    joint_model_group_ = kinematic_model_->getJointModelGroup("manipulator");

    // connect haptic device
    bool device_connected = ConnectHapticDevice();

    if (op_mode == RobotOperateMode::TELEOP_MODE);
    {
        assert(device_connected);
    }
    
    std::vector<double> stretch={1.0, 1.0, 1.0};

    if (GetHapticDeviceHandler()->IsConnected())
    {
       GetHapticDeviceHandler()->SetStretch(stretch);
    }

}


void TeleopVirtualRobot::start()
{
    RegisterCallback();


//  keep alive
    EnableRobot();

    listen_input_thread_ = std::make_shared<std::thread>(
      boost::bind(&TeleopVirtualRobot::ListenInputThreadRun, this));

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok() && (keepalive_ == true)) {
        ros::Duration(0.001).sleep();
        // ROS_INFO("working in main loop");
        //  usleep(1000);
    } 

    DisableRobot();

}


TeleopVirtualRobot::~TeleopVirtualRobot()
{
    DisableRobot();
}


// callback member function
void TeleopVirtualRobot::RegisterCallback()
{
    sub_joint_state_ = nh_.subscribe("joint_states", 20, &TeleopVirtualRobot::JointStatesCallback, this);
    send_command_timer_ = nh_.createTimer(ros::Duration(0.008), &TeleopVirtualRobot::SendCommandTimer, this);
}


void TeleopVirtualRobot::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg)
{
    val_lock_.lock();

    // joint position
    joint_states_msg_ = *joint_msg;

    auto it = joint_msg->name.begin();

    // dump data
    // not using hardcode, but will decrease the efficience
    for (size_t i = 0; i < tool_joints_name_.size(); i++)
    {
        it=std::find(joint_msg->name.begin(), joint_msg->name.end(),tool_joints_name_[i]);
        if (it != joint_msg->name.end())
        {
            joints_position_[tool_joints_name_[i]] = joint_msg->position[it -joint_msg->name.begin()];
            joints_velocity_[tool_joints_name_[i]] = joint_msg->velocity[it-joint_msg->name.begin()];
            joints_effort_[tool_joints_name_[i]] = joint_msg->effort[it-joint_msg->name.begin()];
        }
    }

    for (size_t i = 0; i < arm_joints_name_.size(); i++)
    {
        it=std::find(joint_msg->name.begin(), joint_msg->name.end(),arm_joints_name_[i]);
        if (it != joint_msg->name.end())
        {
            joints_position_[arm_joints_name_[i]] = joint_msg->position[it -joint_msg->name.begin()];
            joints_velocity_[arm_joints_name_[i]] = joint_msg->velocity[it-joint_msg->name.begin()];
            joints_effort_[arm_joints_name_[i]] = joint_msg->effort[it-joint_msg->name.begin()];
        }
    }

    val_lock_.unlock();
}


void TeleopVirtualRobot::ToolWrechCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
{

}


geometry_msgs::PoseStamped TeleopVirtualRobot::GetCurrentArmEndPose()
{
    // current end effector link is wrist_3_link in group "manipulator"
    return arm_commander_.getCurrentPose(arm_commander_.getEndEffectorLink());
}

// void TeleopVirtualRobot::ToolPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& tool_msg)
// {

// }

// void TeleopVirtualRobot::ToolVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
// {

// }

// operate
void TeleopVirtualRobot::SendCommandTimer(const ros::TimerEvent& event)
{
 // delta handler pose add the end tool pose
  // get the new data from handler
  std::vector<double> new_position{0.0, 0.0, 0.0};
  double haptic_end_move_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Eigen::Affine3d end_pose_command = Eigen::Affine3d::Identity();
  double end_velocity_cmd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (!haptic_dev_->IsLocked()) {
      
      if (haptic_dev_->IsConnected()) {
      // for TELEOP_MODE
      new_position = haptic_dev_->GetEndPosition();

      val_lock_.lock();
      for (int i=0;i<3;i++) {
        haptic_end_move_pose[i] =  (new_position[i] - haptic_unlock_data_[i]) * haptic_dev_->GetStretch()[i];
      }

      haptic_unlock_data_ = new_position;
      val_lock_.unlock();

      ROS_INFO("get handler pos:[%1.5f, %1.5f, %1.5f]", new_position[0], new_position[1], new_position[2]);

      // map_pos[0] = -send[1]; // left, right
      // map_pos[2] = send[0];  // forward, backward
      // map_pos[1] = -send[2]; // up, down

      end_pose_command.translation() = Eigen::Vector3d(haptic_end_move_pose[1], 
                                                        - haptic_end_move_pose[0],
                                                         haptic_end_move_pose[2]);

      // We use sample_period as velocity computing time, since we always
      // get/send position difference with the nearest sample data. 

      end_velocity_cmd[0] = end_pose_command.translation()(0)/command_send_period_;
      end_velocity_cmd[1] = end_pose_command.translation()(1)/command_send_period_;
      end_velocity_cmd[2] = end_pose_command.translation()(2)/command_send_period_;

      SendPoseCmdToRobot(end_pose_command);

      } else {
        // for AUTO_MODE
        // if (operate_mode_ == AUTO_MODE)
        // {
        //           doAutonomyOP();
        // }
      }
    }
}


void TeleopVirtualRobot::SendPoseCmdToRobot(Eigen::Affine3d end_pose_cmd)
{
    ROS_INFO("init end pose cmd is: %f , %f , %f", end_pose_cmd.translation()(0), end_pose_cmd.translation()(1), end_pose_cmd.translation()(2));
    std::map<std::string, double> arm_joint_position;
    std::map<std::string, double> arm_joint_vel;
    std::map<std::string, double> arm_joint_acc;

    for (size_t i = 0; i < arm_joints_name_.size(); i++)
    {
        arm_joint_position[arm_joints_name_[i]] =  0.0;
        arm_joint_vel[arm_joints_name_[i]] =  0.0;
        arm_joint_acc[arm_joints_name_[i]] =  0.0;
    }

    // Get current robot end pose
    auto current_end_pose = GetCurrentArmEndPose();
    ROS_INFO("current pose is %f, %f, %f", current_end_pose.pose.position.x,  current_end_pose.pose.position.y,  current_end_pose.pose.position.z);


    // end_pose_cmd = end_pose_cmd + current_end_pose;
    end_pose_cmd.translation()(0) = current_end_pose.pose.position.x + end_pose_cmd.translation()(0);
    end_pose_cmd.translation()(1) = current_end_pose.pose.position.y + end_pose_cmd.translation()(1);
    end_pose_cmd.translation()(2) = current_end_pose.pose.position.z + end_pose_cmd.translation()(2);

    Eigen::Quaterniond rot(current_end_pose.pose.orientation.w, current_end_pose.pose.orientation.x,
                           current_end_pose.pose.orientation.y, current_end_pose.pose.orientation.z);
    end_pose_cmd.linear() = rot.toRotationMatrix() * end_pose_cmd.linear();
   
    // using arm IK solver with moveit
    bool found_ik_solution = GetArmIK(end_pose_cmd, arm_joint_position);

    if (found_ik_solution) {
        for (size_t i = 0; i < arm_joints_name_.size(); i++)
        {
            // TODO: should not this velocity
            arm_joint_vel[arm_joints_name_[i]] = arm_joint_position[arm_joints_name_[i]]/command_send_period_ ;
        }

        ROS_INFO("we get the IK solution");

        // send to robot using robot joint controller
        SendJointTrajectoryCmd(arm_joint_position, arm_joint_vel, arm_joint_acc);
    }
    
}


bool TeleopVirtualRobot::GetArmIK(Eigen::Affine3d end_pose, std::map<std::string, double> & ret)
{
   // print model frame
   ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    val_lock_.lock();
    kinematic_state_->setVariableValues(joint_states_msg_); // update current state
    val_lock_.unlock();

    kinematic_state_->update();

    std::vector<double> joint_values;
    const std::vector<std::string> joint_names = joint_model_group_->getJointModelNames();


    // do inverse kinematics
    bool found_ik = kinematic_state_->setFromIK(joint_model_group_, end_pose, 10, 0.1);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            ROS_INFO("We get inverse Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            ROS_INFO("The current joint %s: %f", joint_names[i].c_str(), joints_position_[joint_names[i]]);
        }

        for (std::size_t i = 0; i < joint_names.size(); i++)
        {
            ret[joint_names[i]] = joint_values[i];
        }
    
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }


    return found_ik;
}


void TeleopVirtualRobot::SendJointTrajectoryCmd(std::map<std::string, double> position, std::map<std::string, double> vel, std::map<std::string, double> acc)
{
    // using joint trajectory controller
    // we will send these joint value to the arm in gazebo use arm_controller

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  control_msgs::FollowJointTrajectoryGoal goal;

  for (auto it=position.begin(); it != position.end(); it++)
  {
      goal.trajectory.joint_names.push_back(it->first);
  }
  
  goal.trajectory.points.resize(1);

   int index = 0;
   goal.trajectory.points[index].positions.resize(position.size());
   goal.trajectory.points[index].velocities.resize(position.size());

  for (size_t i = 0; i < arm_joints_name_.size(); i++)
  {
    goal.trajectory.points[index].positions[i] = position[goal.trajectory.joint_names[i]];

    goal.trajectory.points[index].velocities[i] = vel[goal.trajectory.joint_names[i]];
  }

  goal.trajectory.points[index].time_from_start = ros::Duration(control_executation_period_); // duration for executing the trajectory

  ac_.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac_.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_.getState();
  }
  else
    ROS_INFO("Action did not finish before the time out.");
}


// void TeleopVirtualRobot::DoAutonomyOP();

// // cartesian space move
// int TeleopVirtualRobot::MoveEndSpeedL(double end_relative_pose[]);
// int TeleopVirtualRobot::MoveEndL(double pose[], double a=1.2, double v=0.25, double t=0.02, double r=0);
// int TeleopVirtualRobot::MoveEndRelativeL(double pose[], double a=1.2, double v=0.25, double t=0.02, double r=0);
// int TeleopVirtualRobot::SetToolPose(double tool_pose[], int length);

// // joint space move
// int TeleopVirtualRobot::MoveServoj(double pose[], double a, double v, double t=0);

// // get robot states
// int TeleopVirtualRobot::GetCurrentJointStates(double angle[], int length);
// int TeleopVirtualRobot::GetCurrentWrenchState(double wrech[], int length);
// int TeleopVirtualRobot::GetToolPose(double tool_pose[], int length);

// // stop
// int TeleopVirtualRobot::halt(); // join the listenHandlerThread

// // lock
// void TeleopVirtualRobot::setHandlerLockFlag(bool flag);
// bool TeleopVirtualRobot::isHandlerLocked()
// {

// }

void TeleopVirtualRobot::SendSafetyCommand()
{
    double init_vel[6]={0.0, 0.0, 0.0,0.0, 0.0, 0.0};
    //  speedL(init_vel);
}


void TeleopVirtualRobot::ListenInputThreadRun()
{
   
   ROS_INFO("listen input thread run");
  if (ros::ok() && keepalive_ ) {
    static int count = 0;
    double a[3] = {0.0, 0.0, 0.0};
    char keyboard_input;

    while(ros::ok() && 'q' != (keyboard_input=getKbhit())) {
      if (keyboard_input == 'p') {
        count = (count+1)%2;
        SendSafetyCommand();
        if (count%2) {

            val_lock_.lock();
            haptic_unlock_data_ =  haptic_dev_->GetEndPosition();
            val_lock_.unlock();

            haptic_dev_->SetLocked(false);
           ROS_INFO("Start getting handler data...");
        } else {
           haptic_dev_->SetLocked(true);
           ROS_INFO("Stop getting handler data!");
        }
      }

      // for avoiding cpu 100%
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    // we will exit this thread, we should keep all move stop!
    SendSafetyCommand();
    ROS_INFO("Exit listen handler data");
    haptic_dev_->SetLocked(true);

    // let's shutdown everything!
    keepalive_ = false;
    ros::shutdown();
  }
}

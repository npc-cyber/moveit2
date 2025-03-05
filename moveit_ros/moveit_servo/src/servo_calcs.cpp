/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : servo_calcs.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <cassert>
#include <thread>
#include <chrono>
#include <mutex>

#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <moveit_servo/make_shared_from_pool.h> // TODO(adamp): create an issue about this
#include <moveit_servo/enforce_limits.hpp>
#include <moveit_servo/servo_calcs.h>
#include <moveit_servo/utilities.h>

using namespace std::chrono_literals;  // for s, ms, etc.

namespace moveit_servo
{
namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_calcs");
constexpr auto ROS_LOG_THROTTLE_PERIOD = std::chrono::milliseconds(3000).count();
static constexpr double STOPPED_VELOCITY_EPS = 1e-4;  // rad/s
}  // namespace

// Constructor for the class that handles servoing calculations
ServoCalcs::ServoCalcs(const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<const moveit_servo::ServoParameters>& parameters,
                       const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , stop_requested_(true)
  , done_stopping_(false)
  , paused_(false)
  , robot_link_command_frame_(parameters->robot_link_command_frame)
  , smoothing_loader_("moveit_core", "online_signal_smoothing::SmoothingBaseClass")
{
  // Register callback for changes in robot_link_command_frame
  bool callback_success = parameters_->registerSetParameterCallback(parameters->ns + ".robot_link_command_frame",
                                                                    [this](const rclcpp::Parameter& parameter) {
                                                                      return robotLinkCommandFrameCallback(parameter);
                                                                    });
  if (!callback_success)
  {
    throw std::runtime_error("Failed to register setParameterCallback");
  }

  // MoveIt Setup
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = current_state_->getJointModelGroup(parameters_->move_group_name);
  if (joint_model_group_ == nullptr)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid move group name: `" << parameters_->move_group_name << "`");
    throw std::runtime_error("Invalid move group name");
  }

  // Subscribe to command topics
  // 这个地方是订阅消息的地方
  // 可以看到这里就订阅了两个与操控机械臂有关的消息
  // 一个传入末端线速度  一个传入关节角速度
  twist_stamped_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      parameters_->cartesian_command_in_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) { return twistStampedCB(msg); });

  joint_cmd_sub_ = node_->create_subscription<control_msgs::msg::JointJog>(
      parameters_->joint_command_in_topic, rclcpp::SystemDefaultsQoS(),
      [this](const control_msgs::msg::JointJog::ConstSharedPtr& msg) {
        // 这个地方只负责多线程记录消息 latest_joint_cmd_
        // class 的函数 负责处理latest_joint_cmd_
        return jointCmdCB(msg);
      });

  // ROS Server for allowing drift in some dimensions
  drift_dimensions_server_ = node_->create_service<moveit_msgs::srv::ChangeDriftDimensions>(
      "~/change_drift_dimensions",
      [this](const std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Response>& res) {
        return changeDriftDimensions(req, res);
      });

  // ROS Server for changing the control dimensions
  control_dimensions_server_ = node_->create_service<moveit_msgs::srv::ChangeControlDimensions>(
      "~/change_control_dimensions",
      [this](const std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Response>& res) {
        return changeControlDimensions(req, res);
      });

  // ROS Server to reset the status, e.g. so the arm can move again after a collision
  reset_servo_status_ = node_->create_service<std_srvs::srv::Empty>(
      "~/reset_servo_status",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>& req,
             const std::shared_ptr<std_srvs::srv::Empty::Response>& res) { return resetServoStatus(req, res); });

  // Subscribe to the collision_check topic
  collision_velocity_scale_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/collision_velocity_scale", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::ConstSharedPtr& msg) { return collisionVelocityScaleCB(msg); });

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
  if (parameters_->command_out_type == "trajectory_msgs/JointTrajectory")
  {
    trajectory_outgoing_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        parameters_->command_out_topic, rclcpp::SystemDefaultsQoS());
  }
  else if (parameters_->command_out_type == "std_msgs/Float64MultiArray")
  {
    multiarray_outgoing_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        parameters_->command_out_topic, rclcpp::SystemDefaultsQoS());
  }

  // Publish status
  status_pub_ = node_->create_publisher<std_msgs::msg::Int8>(parameters_->status_topic, rclcpp::SystemDefaultsQoS());

  internal_joint_state_.name = joint_model_group_->getActiveJointModelNames();
  num_joints_ = internal_joint_state_.name.size();
  internal_joint_state_.position.resize(num_joints_);
  internal_joint_state_.velocity.resize(num_joints_);
  delta_theta_.setZero(num_joints_);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // A map for the indices of incoming joint commands
    joint_state_name_map_[internal_joint_state_.name[i]] = i;
  }

  // Load the smoothing plugin
  try
  {
    smoother_ = smoothing_loader_.createSharedInstance(parameters_->smoothing_filter_plugin_name);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading the smoothing plugin '%s': '%s'",
                 parameters_->smoothing_filter_plugin_name.c_str(), ex.what());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the smoothing plugin
  if (!smoother_->initialize(node_, planning_scene_monitor_->getRobotModel(), num_joints_))
  {
    RCLCPP_ERROR(LOGGER, "Smoothing plugin could not be initialized");
    std::exit(EXIT_FAILURE);
  }

  // A matrix of all zeros is used to check whether matrices have been initialized
  Eigen::Matrix3d empty_matrix;
  empty_matrix.setZero();
  tf_moveit_to_ee_frame_ = empty_matrix;
  tf_moveit_to_robot_cmd_frame_ = empty_matrix;

  // Get the IK solver for the group
  ik_solver_ = joint_model_group_->getSolverInstance();
  if (!ik_solver_)
  {
    use_inv_jacobian_ = true;
    RCLCPP_WARN(
        LOGGER,
        "No kinematics solver instantiated for group '%s'. Will use inverse Jacobian for servo calculations instead.",
        joint_model_group_->getName().c_str());
  }
  else if (!ik_solver_->supportsGroup(joint_model_group_))
  {
    use_inv_jacobian_ = true;
    RCLCPP_WARN(LOGGER,
                "The loaded kinematics plugin does not support group '%s'. Will use inverse Jacobian for servo "
                "calculations instead.",
                joint_model_group_->getName().c_str());
  }
  // use_inv_jacobian_ = true;
  RCLCPP_DEBUG(LOGGER, (std::string("find pose") + std::string(use_inv_jacobian_ ? "use_inv_jacobian " : "not use_inv_jacobian")).c_str());
}

ServoCalcs::~ServoCalcs()
{
  stop();
}

void ServoCalcs::start()
{
  // Stop the thread if we are currently running
  stop();

  // Set up the "last" published message, in case we need to send it first
  auto initial_joint_trajectory = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  initial_joint_trajectory->header.stamp = node_->now();
  initial_joint_trajectory->header.frame_id = parameters_->planning_frame;
  initial_joint_trajectory->joint_names = internal_joint_state_.name;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(parameters_->publish_period);
  if (parameters_->publish_joint_positions)
    planning_scene_monitor_->getStateMonitor()->getCurrentState()->copyJointGroupPositions(joint_model_group_,
                                                                                           point.positions);
  if (parameters_->publish_joint_velocities)
  {
    std::vector<double> velocity(num_joints_);
    point.velocities = velocity;
  }
  if (parameters_->publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    point.accelerations.resize(num_joints_);
  }
  initial_joint_trajectory->points.push_back(point);
  last_sent_command_ = std::move(initial_joint_trajectory);

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_->ee_frame_name);
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(robot_link_command_frame_);
  if (!use_inv_jacobian_)
  {
    ik_base_to_tip_frame_ = current_state_->getGlobalLinkTransform(ik_solver_->getBaseFrame()).inverse() *
                            current_state_->getGlobalLinkTransform(ik_solver_->getTipFrame());
  }

  stop_requested_ = false;
  thread_ = std::thread([this] { mainCalcLoop(); });
  new_input_cmd_ = false;
}

void ServoCalcs::stop()
{
  // Request stop
  stop_requested_ = true;

  // Notify condition variable in case the thread is blocked on it
  {
    // scope so the mutex is unlocked after so the thread can continue
    // and therefore be joinable
    const std::lock_guard<std::mutex> lock(main_loop_mutex_);
    new_input_cmd_ = false;
    input_cv_.notify_all();
  }

  // Join the thread
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void ServoCalcs::mainCalcLoop()
{
  rclcpp::WallRate rate(1.0 / parameters_->publish_period);

  while (rclcpp::ok() && !stop_requested_)
  {
    // lock the input state mutex
    std::unique_lock<std::mutex> main_loop_lock(main_loop_mutex_);

    // low latency mode -- begin calculations as soon as a new command is received.
    if (parameters_->low_latency_mode)
    {
      input_cv_.wait(main_loop_lock, [this] { return (new_input_cmd_ || stop_requested_); });
    }

    // reset new_input_cmd_ flag
    new_input_cmd_ = false;

    // run servo calcs
    const auto start_time = node_->now();
    calculateSingleIteration();
    const auto run_duration = node_->now() - start_time;

    // Log warning when the run duration was longer than the period
    if (run_duration.seconds() > parameters_->publish_period)
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "run_duration: " << run_duration.seconds() << " (" << parameters_->publish_period
                                                   << ")");
    }

    // normal mode, unlock input mutex and wait for the period of the loop
    if (!parameters_->low_latency_mode)
    {
      main_loop_lock.unlock();
      rate.sleep();
    }
  }
}

void ServoCalcs::calculateSingleIteration()
{
  // 这个地方是具体执行动作的地方
  // Publish status each loop iteration
  auto status_msg = std::make_unique<std_msgs::msg::Int8>();
  status_msg->data = static_cast<int8_t>(status_);
  status_pub_->publish(std::move(status_msg));

  // After we publish, status, reset it back to no warnings
  status_ = StatusCode::NO_WARNING;

  // Always update the joints and end-effector transform for 2 reasons:
  // 1) in case the getCommandFrameTransform() method is being used
  // 2) so the low-pass filters are up to date and don't cause a jump
  updateJoints();

  // Update from latest state
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

  if (latest_twist_stamped_)
    twist_stamped_cmd_ = *latest_twist_stamped_;
  if (latest_joint_cmd_)
    joint_servo_cmd_ = *latest_joint_cmd_;

  // Check for stale cmds
  // 如果距离上次时间较长 可以更新这个地方为true
  twist_command_is_stale_ = ((node_->now() - latest_twist_command_stamp_) >=
                             rclcpp::Duration::from_seconds(parameters_->incoming_command_timeout));
  joint_command_is_stale_ = ((node_->now() - latest_joint_command_stamp_) >=
                             rclcpp::Duration::from_seconds(parameters_->incoming_command_timeout));

  have_nonzero_twist_stamped_ = latest_twist_cmd_is_nonzero_;
  have_nonzero_joint_command_ = latest_joint_cmd_is_nonzero_;

  // Get the transform from MoveIt planning frame to servoing command frame
  // Calculate this transform to ensure it is available via C++ API
  // We solve (planning_frame -> base -> robot_link_command_frame)
  // by computing (base->planning_frame)^-1 * (base->robot_link_command_frame)
  // 这几个 tf 需要关注一下 后续看ee link 规划时需要用到这个
  // tf 也没问题啊 为什么坐标轴不对
  // {
  //   Eigen::Isometry3d transform = current_state_->getGlobalLinkTransform(parameters_->planning_frame);
  //   Eigen::Quaterniond quaternion(transform.rotation());
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   Eigen::Vector3d translation = transform.translation();
  //   RCLCPP_WARN(LOGGER, "planning_frame Quaternion Translation: (%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f)",
  //               quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(), translation.x(), translation.y(),
  //               translation.z());
  // }

  // {
  //   // 这个地方是 eelink到world的
  //   // 和rviz的一样
  //   // 也就是 P_world = RT_ee_2_world * P_ee 这个已经确认过了 没有错误
  //   Eigen::Isometry3d transform = current_state_->getGlobalLinkTransform(parameters_->ee_frame_name);
  //   Eigen::Quaterniond quaternion(transform.rotation());
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   Eigen::Vector3d translation = transform.translation();
  //   RCLCPP_WARN(LOGGER, "ee_frame_name Quaternion Translation: (%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f)",
  //               quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(), translation.x(), translation.y(),
  //               translation.z());
  // }

  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(robot_link_command_frame_);

  // Calculate the transform from MoveIt planning frame to End Effector frame
  // Calculate this transform to ensure it is available via C++ API
  // 这个地方就是 RT_eelink_to_planing_frame
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_->ee_frame_name);

  // {
  //   // RT_eelink_to_planing_frame
  //   Eigen::Isometry3d transform = tf_moveit_to_ee_frame_;
  //   Eigen::Quaterniond quaternion(transform.rotation());
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   Eigen::Vector3d translation = transform.translation();
  //   RCLCPP_WARN(LOGGER, "tf_moveit_to_ee_frame_ Quaternion Translation: (%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f)",
  //               quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(), translation.x(), translation.y(),
  //               translation.z());
  // }

  if (!use_inv_jacobian_)
  {
    // 这个是 RT_tip_frame_to_base
    ik_base_to_tip_frame_ = current_state_->getGlobalLinkTransform(ik_solver_->getBaseFrame()).inverse() *
                            current_state_->getGlobalLinkTransform(ik_solver_->getTipFrame());
  }

  have_nonzero_command_ = have_nonzero_twist_stamped_ || have_nonzero_joint_command_;

  // Don't end this function without updating the filters
  updated_filters_ = false;

  // If paused or while waiting for initial servo commands, just keep the low-pass filters up to date with current
  // joints so a jump doesn't occur when restarting
  if (wait_for_servo_commands_ || paused_)
  {
    resetLowPassFilters(original_joint_state_);

    // Check if there are any new commands with valid timestamp
    wait_for_servo_commands_ =
        twist_stamped_cmd_.header.stamp == rclcpp::Time(0.) && joint_servo_cmd_.header.stamp == rclcpp::Time(0.);

    // Early exit
    return;
  }

  // If not waiting for initial command, and not paused.
  // Do servoing calculations only if the robot should move, for efficiency
  // Create new outgoing joint trajectory command message
  auto joint_trajectory = std::make_unique<trajectory_msgs::msg::JointTrajectory>();

  // Prioritize cartesian servoing above joint servoing
  // Only run commands if not stale and nonzero
  if (have_nonzero_twist_stamped_ && !twist_command_is_stale_)
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "begin cartesian servoing");
    if (!cartesianServoCalcs(twist_stamped_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else if (have_nonzero_joint_command_ && !joint_command_is_stale_)
  {
    // 上面的不管 这个地方我们获取到了轨迹
    if (!jointServoCalcs(joint_servo_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else
  {
    // 这个地方就是不动 角度改成 last_sent_command_ 速度改为0 就是不动
    // Joint trajectory is not populated with anything, so set it to the last positions and 0 velocity
    *joint_trajectory = *last_sent_command_;
    for (auto& point : joint_trajectory->points)
    {
      point.velocities.assign(point.velocities.size(), 0);
    }
  }

  // Print a warning to the user if both are stale
  if (twist_command_is_stale_ && joint_command_is_stale_)
  {
    // 最终都是传入一个关节角的轨迹
    filteredHalt(*joint_trajectory);
  }
  else
  {
    done_stopping_ = false;
  }

  // Skip the servoing publication if all inputs have been zero for several cycles in a row.
  // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep republishing forever.
  if (!have_nonzero_command_ && done_stopping_ && (parameters_->num_outgoing_halt_msgs_to_publish != 0) &&
      (zero_velocity_count_ > parameters_->num_outgoing_halt_msgs_to_publish))
  {
    ok_to_publish_ = false;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "All-zero command. Doing nothing.");
  }
  // Skip servoing publication if both types of commands are stale.
  else if (twist_command_is_stale_ && joint_command_is_stale_)
  {
    ok_to_publish_ = false;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                 "Skipping publishing because incoming commands are stale.");
  }
  else
  {
    ok_to_publish_ = true;
  }

  // Store last zero-velocity message flag to prevent superfluous warnings.
  // Cartesian and joint commands must both be zero.
  if (!have_nonzero_command_ && done_stopping_)
  {
    // Avoid overflow
    if (zero_velocity_count_ < std::numeric_limits<int>::max())
      ++zero_velocity_count_;
  }
  else
  {
    zero_velocity_count_ = 0;
  }

  if (ok_to_publish_ && !paused_)
  {
    // Clear out position commands if user did not request them (can cause interpolation issues)
    if (!parameters_->publish_joint_positions)
    {
      joint_trajectory->points[0].positions.clear();
    }
    // Likewise for velocity and acceleration
    if (!parameters_->publish_joint_velocities)
    {
      joint_trajectory->points[0].velocities.clear();
    }
    if (!parameters_->publish_joint_accelerations)
    {
      joint_trajectory->points[0].accelerations.clear();
    }

    // Put the outgoing msg in the right format
    // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
    if (parameters_->command_out_type == "trajectory_msgs/JointTrajectory")
    {
      // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
      // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
      joint_trajectory->header.stamp = rclcpp::Time(0);
      *last_sent_command_ = *joint_trajectory;
      // 在这个地方发送关节角 我踏马的 这个又是给谁接收的呢
      // for (std::size_t i = 0; i < joint_trajectory->points.size(); ++i)
      // {
      //   // 这个地方有问题 只有一个关节要动 多个关节都发送了数据
      //   rclcpp::Clock& clock = *node_->get_clock();
      //   RCLCPP_WARN_STREAM_THROTTLE(
      //       LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
      //       "topic echo change joint delta "
      //           << joint_trajectory->points.size() << " || " << i << " || " << joint_trajectory->points[i].positions[0]
      //           << " || " << joint_trajectory->points[i].positions[1] << " || "
      //           << joint_trajectory->points[i].positions[2] << " || " << joint_trajectory->points[i].positions[3]
      //           << " || " << joint_trajectory->points[i].positions[4] << " || "
      //           << joint_trajectory->points[i].positions[5] << " || " << joint_trajectory->points[i].positions[6]);
      // }
      // 其实就传了一个关节 但是其他关节有数值误差
      trajectory_outgoing_cmd_pub_->publish(std::move(joint_trajectory));
    }
    else if (parameters_->command_out_type == "std_msgs/Float64MultiArray")
    {
      auto joints = std::make_unique<std_msgs::msg::Float64MultiArray>();
      if (parameters_->publish_joint_positions && !joint_trajectory->points.empty())
        joints->data = joint_trajectory->points[0].positions;
      else if (parameters_->publish_joint_velocities && !joint_trajectory->points.empty())
        joints->data = joint_trajectory->points[0].velocities;
      *last_sent_command_ = *joint_trajectory;
      multiarray_outgoing_cmd_pub_->publish(std::move(joints));
    }
  }

  // Update the filters if we haven't yet
  if (!updated_filters_)
    resetLowPassFilters(original_joint_state_);
}

rcl_interfaces::msg::SetParametersResult ServoCalcs::robotLinkCommandFrameCallback(const rclcpp::Parameter& parameter)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  robot_link_command_frame_ = parameter.as_string();
  RCLCPP_INFO_STREAM(LOGGER, "robot_link_command_frame changed to: " + robot_link_command_frame_);
  return result;
};

// Perform the servoing calculations
bool ServoCalcs::cartesianServoCalcs(geometry_msgs::msg::TwistStamped& cmd,
                                     trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // Check for nan's in the incoming command
  if (!checkValidCommand(cmd))
    return false;

  // Set uncontrolled dimensions to 0 in command frame
  // The dimensions to control. In the command frame. [x, y, z, roll, pitch, yaw]
  // 传入的是 坐标增量 和 欧拉角增量 ?
  // 其实不是的 传入的还是 末端坐标速度 和 末端欧拉角速度
  // scaleCartesianCommand() 函数可以解释
  enforceControlDimensions(cmd);

  // Transform the command to the MoveGroup planning frame
  if (cmd.header.frame_id != parameters_->planning_frame)
  {
    Eigen::Vector3d translation_vector(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);
    Eigen::Vector3d angular_vector(cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z);

    // 看来是把速度转换到 planning frame 上
    // If the incoming frame is empty or is the command frame, we use the previously calculated tf
    if (cmd.header.frame_id.empty() || cmd.header.frame_id == robot_link_command_frame_)
    {
      translation_vector = tf_moveit_to_robot_cmd_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_robot_cmd_frame_.linear() * angular_vector;
    }
    else if (cmd.header.frame_id == parameters_->ee_frame_name)
    {
      // If the frame is the EE frame, we already have that transform as well
      translation_vector = tf_moveit_to_ee_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_ee_frame_.linear() * angular_vector;
    }
    else
    {
      // We solve (planning_frame -> base -> cmd.header.frame_id)
      // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
      const auto tf_moveit_to_incoming_cmd_frame =
          current_state_->getGlobalLinkTransform(parameters_->planning_frame).inverse() *
          current_state_->getGlobalLinkTransform(cmd.header.frame_id);
      translation_vector = tf_moveit_to_incoming_cmd_frame.linear() * translation_vector;
      angular_vector = tf_moveit_to_incoming_cmd_frame.linear() * angular_vector;
    }
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                "begin cartesian servoing " << parameters_->planning_frame);
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "cmd.header.frame_id " << cmd.header.frame_id);

    // Put these components back into a TwistStamped
    cmd.header.frame_id = parameters_->planning_frame;
    cmd.twist.linear.x = translation_vector(0);
    cmd.twist.linear.y = translation_vector(1);
    cmd.twist.linear.z = translation_vector(2);
    cmd.twist.angular.x = angular_vector(0);
    cmd.twist.angular.y = angular_vector(1);
    cmd.twist.angular.z = angular_vector(2);
  }

  {
    // 从这个地方看也是正确的
    // tf监听是对的 最后的dx也是对的
    // 难道是jacobian的计算有问题?
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_INFO_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                "final cmd prar " << "frame id " << cmd.header.frame_id << "x " << cmd.twist.linear.x
                                                  << " y " << cmd.twist.linear.y << " z " << cmd.twist.linear.z
                                                  << "roll " << cmd.twist.angular.x << " pitch " << cmd.twist.angular.y
                                                  << " yaw " << cmd.twist.angular.z);
  }

  // scaleCartesianCommand 明显又乘以时间了
  Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);

  Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);

  // 我明白了 这个地方类似于之前matlab的相机标定 把不能优化的变量锁住 不让有增量delta
  removeDriftDimensions(jacobian, delta_x);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  // Convert from cartesian commands to joint commands
  // Use an IK solver plugin if we have one, otherwise use inverse Jacobian.
  if (!use_inv_jacobian_)
  {
    // get a transformation matrix with the desired position change &
    // get a transformation matrix with desired orientation change
    Eigen::Isometry3d tf_pos_delta(Eigen::Isometry3d::Identity());
    tf_pos_delta.translate(Eigen::Vector3d(delta_x[0], delta_x[1], delta_x[2]));

    Eigen::Isometry3d tf_rot_delta(Eigen::Isometry3d::Identity());
    Eigen::Quaterniond q = Eigen::AngleAxisd(delta_x[3], Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(delta_x[4], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(delta_x[5], Eigen::Vector3d::UnitZ());
    tf_rot_delta.rotate(q);

    // Poses passed to IK solvers are assumed to be in some tip link (usually EE) reference frame
    // First, find the new tip link position without newly applied rotation

    // bug_fix : 原因找到了 我们有两个base_link 和一个 world 没有对齐
    // tf_pos_delta 是 world 上的增量  不等base_libk上的增量
    // RT_tip_frame_to_base
    auto tf_no_new_rot = tf_pos_delta * ik_base_to_tip_frame_;
    // we want the rotation to be applied in the requested reference frame,
    // but we want the rotation to be about the EE point in space, not the origin.
    // So, we need to translate to origin, rotate, then translate back
    // Given T = transformation matrix from origin -> EE point in space (translation component of tf_no_new_rot)
    // and T' as the opposite transformation, EE point in space -> origin (translation only)
    // apply final transformation as T * R * T' * tf_no_new_rot
    auto tf_translation = tf_no_new_rot.translation();
    auto tf_neg_translation = Eigen::Isometry3d::Identity();  // T'
    tf_neg_translation(0, 3) = -tf_translation(0, 0);
    tf_neg_translation(1, 3) = -tf_translation(1, 0);
    tf_neg_translation(2, 3) = -tf_translation(2, 0);
    auto tf_pos_translation = Eigen::Isometry3d::Identity();  // T
    tf_pos_translation(0, 3) = tf_translation(0, 0);
    tf_pos_translation(1, 3) = tf_translation(1, 0);
    tf_pos_translation(2, 3) = tf_translation(2, 0);

    // T * R * T' * tf_no_new_rot
    auto tf = tf_pos_translation * tf_rot_delta * tf_neg_translation * tf_no_new_rot;
    geometry_msgs::msg::Pose next_pose = tf2::toMsg(tf);

    {
      // 从这个地方看也是正确的
      // tf监听是对的 最后的dx也是对的
      // 难道是jacobian的计算有问题?
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "next_pose " << next_pose.position.x << " " << next_pose.position.y << " "
                                               << next_pose.position.z << next_pose.orientation.x << " "
                                               << next_pose.orientation.y << " " << next_pose.orientation.z << " "
                                               << next_pose.orientation.w);
    }

    // setup for IK call
    std::vector<double> solution(num_joints_);
    moveit_msgs::msg::MoveItErrorCodes err;
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true;
    // searchPositionIK 也是虚函数 需要各个IK插件重写
    // 这个也是求运动学逆解的
    if (ik_solver_->searchPositionIK(next_pose, internal_joint_state_.position, parameters_->publish_period / 2.0,
                                     solution, err, opts))
    {
      // find the difference in joint positions that will get us to the desired pose
      for (size_t i = 0; i < num_joints_; ++i)
      {
        delta_theta_.coeffRef(i) = solution.at(i) - internal_joint_state_.position.at(i);
      }
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Could not find IK solution for requested motion, got error code %d", err.val);
      return false;
    }
  }
  else
  {
    // no supported IK plugin, use inverse Jacobian
    // 直接用 use inverse Jacobian 得到关节增量 也就是根据末端增量得到关节增量
    // 也就是那个公式 https://www.zhihu.com/question/531815112/answer/2476607222

    // ros中做机械臂笛卡尔空间规划如何避免路径突变 博士也是调参
    // https://www.zhihu.com/question/398666486/answer/1258960217

    // https://www.zhihu.com/question/67687838/answer/2063760318 这个必须看一下
    delta_theta_ = pseudo_inverse * delta_x;
  }

  // 打印 delta_theta_ 到一行
  // {
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   RCLCPP_INFO_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "point 1 delta_theta_: " << delta_theta_.transpose());
  // }
  delta_theta_ *= velocityScalingFactorForSingularity(joint_model_group_, delta_x, svd, pseudo_inverse,
                                                      parameters_->hard_stop_singularity_threshold,
                                                      parameters_->lower_singularity_threshold,
                                                      parameters_->leaving_singularity_threshold_multiplier,
                                                      *node_->get_clock(), current_state_, status_);
  // // 打印 delta_theta_ 到一行
  // {
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   RCLCPP_INFO_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "point 2 delta_theta_: " << delta_theta_.transpose());
  // }

  // 最终笛卡尔规划 要转到关节规划
  return internalServoUpdate(delta_theta_, joint_trajectory, ServoType::CARTESIAN_SPACE);
}

bool ServoCalcs::jointServoCalcs(const control_msgs::msg::JointJog& cmd,
                                 trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // Check for nan's
  // 这个地方是demo发送的数据 还没有解析成 trajectory
  if (!checkValidCommand(cmd))
    return false;

  // Apply user-defined scaling
  // 简单的速度乘以时间 这个地方得到的是关节角的增量而不是关节角
  delta_theta_ = scaleJointCommand(cmd);

  // Perform internal servo with the command
  return internalServoUpdate(delta_theta_, joint_trajectory, ServoType::JOINT_SPACE);
}

bool ServoCalcs::internalServoUpdate(Eigen::ArrayXd& delta_theta,
                                     trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                                     const ServoType servo_type)
{
  // The order of operations here is:
  // 1. apply velocity scaling for collisions (in the position domain)
  // 2. low-pass filter the position command in applyJointUpdate()
  // 3. calculate velocities in applyJointUpdate()
  // 4. apply velocity limits
  // 5. apply position limits. This is a higher priority than velocity limits, so check it last.

  // Set internal joint state from original
  internal_joint_state_ = original_joint_state_;

  // Apply collision scaling
  double collision_scale = collision_velocity_scale_;
  if (collision_scale > 0 && collision_scale < 1)
  {
    status_ = StatusCode::DECELERATE_FOR_COLLISION;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, SERVO_STATUS_CODE_MAP.at(status_));
  }
  else if (collision_scale == 0)
  {
    status_ = StatusCode::HALT_FOR_COLLISION;
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "Halting for collision!");
  }
  // 这个地方直接对增量进行减速
  // 我比较好奇的一点时 发一次消息是速度一直不变吗
  delta_theta *= collision_scale;

  // Loop thru joints and update them, calculate velocities, and filter
  if (!applyJointUpdate(delta_theta, internal_joint_state_))
    return false;

  // Mark the lowpass filters as updated for this cycle
  updated_filters_ = true;

  // Enforce SRDF velocity limits
  enforceVelocityLimits(joint_model_group_, parameters_->publish_period, internal_joint_state_,
                        parameters_->override_velocity_scaling_factor);

  // Enforce SRDF position limits, might halt if needed, set prev_vel to 0
  const auto joints_to_halt = enforcePositionLimits(internal_joint_state_);
  if (!joints_to_halt.empty())
  {
    status_ = StatusCode::JOINT_BOUND;
    if ((servo_type == ServoType::JOINT_SPACE && !parameters_->halt_all_joints_in_joint_mode) ||
        (servo_type == ServoType::CARTESIAN_SPACE && !parameters_->halt_all_joints_in_cartesian_mode))
    {
      suddenHalt(internal_joint_state_, joints_to_halt);
    }
    else
    {
      suddenHalt(internal_joint_state_, joint_model_group_->getActiveJointModels());
    }
  }

  // compose outgoing message
  composeJointTrajMessage(internal_joint_state_, joint_trajectory);

  // for (std::size_t i = 0; i < joint_trajectory.points.size(); ++i)
  // {
  //   // 这个地方有问题 只有一个关节要动 多个关节都发送了数据
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   RCLCPP_WARN_STREAM_THROTTLE(
  //       LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
  //       "internalServoUpdate1 joint delta "
  //           << joint_trajectory.points.size() << " || " << i << " || " << joint_trajectory.points[i].positions[0]
  //           << " || " << joint_trajectory.points[i].positions[1] << " || " << joint_trajectory.points[i].positions[2]
  //           << " || " << joint_trajectory.points[i].positions[3] << " || " << joint_trajectory.points[i].positions[4]
  //           << " || " << joint_trajectory.points[i].positions[5] << " || " << joint_trajectory.points[i].positions[6]);
  // }

  // Modify the output message if we are using gazebo
  // 应该是时钟周期不同 需要重新计算
  if (parameters_->use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(joint_trajectory, gazebo_redundant_message_count_);
  }

  // for (std::size_t i = 0; i < joint_trajectory.points.size(); ++i)
  // {
  //   // 这个地方有问题 只有一个关节要动 多个关节都发送了数据
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   RCLCPP_WARN_STREAM_THROTTLE(
  //       LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
  //       "internalServoUpdate2 joint delta "
  //           << joint_trajectory.points.size() << " || " << i << " || " << joint_trajectory.points[i].positions[0]
  //           << " || " << joint_trajectory.points[i].positions[1] << " || " << joint_trajectory.points[i].positions[2]
  //           << " || " << joint_trajectory.points[i].positions[3] << " || " << joint_trajectory.points[i].positions[4]
  //           << " || " << joint_trajectory.points[i].positions[5] << " || " << joint_trajectory.points[i].positions[6]);
  // }

  return true;
}

bool ServoCalcs::applyJointUpdate(const Eigen::ArrayXd& delta_theta, sensor_msgs::msg::JointState& joint_state)
{
  // All the sizes must match
  if (joint_state.position.size() != static_cast<std::size_t>(delta_theta.size()) ||
      joint_state.velocity.size() != joint_state.position.size())
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                 "Lengths of output and increments do not match.");
    return false;
  }
  // // 添加debug
  // {
  //   // 到这个地方是正常的 发送一个关节就只有一个关节
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
  //                                 "applyJointUpdate1() change joint delta "
  //                                 << " || " << joint_state.position[0]
  //                                 << " || " << joint_state.position[1]
  //                                 << " || " << joint_state.position[2]
  //                                 << " || " << joint_state.position[3]
  //                                 << " || " << joint_state.position[4]
  //                                 << " || " << joint_state.position[5]
  //                                 << " || " << joint_state.position[6] );
  // }
  for (std::size_t i = 0; i < joint_state.position.size(); ++i)
  {
    // Increment joint
    joint_state.position[i] += delta_theta[i];
  }
  // // 添加debug
  // {
  //   // 到这个地方是正常的 发送一个关节就只有一个关节
  //   rclcpp::Clock& clock = *node_->get_clock();
  //   RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
  //                               "applyJointUpdate2() change joint delta "
  //                               << " || " << delta_theta[0]
  //                               << " || " << delta_theta[1]
  //                               << " || " << delta_theta[2]
  //                               << " || " << delta_theta[3]
  //                               << " || " << delta_theta[4]
  //                               << " || " << delta_theta[5]
  //                               << " || " << delta_theta[6] );
  // }

  smoother_->doSmoothing(joint_state.position);

  for (std::size_t i = 0; i < joint_state.position.size(); ++i)
  {
    // Calculate joint velocity
    // 这个地方不是傻逼吗
    // 之前传入一个关节角速度 计算增量
    // 现在又根据关节角增量再来算速度
    joint_state.velocity[i] =
        (joint_state.position.at(i) - original_joint_state_.position.at(i)) / parameters_->publish_period;
  }

  return true;
}

// Spam several redundant points into the trajectory. The first few may be skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo simulation.
// Start from 1 because the first point's timestamp is already 1*parameters_->publish_period
void ServoCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                                                     int count) const
{
  if (count < 2)
    return;
  joint_trajectory.points.resize(count);
  auto point = joint_trajectory.points[0];
  // Start from 1 because we already have the first point. End at count+1 so (total #) == count
  for (int i = 1; i < count; ++i)
  {
    point.time_from_start = rclcpp::Duration::from_seconds((i + 1) * parameters_->publish_period);
    joint_trajectory.points[i] = point;
  }
}

void ServoCalcs::resetLowPassFilters(const sensor_msgs::msg::JointState& joint_state)
{
  smoother_->reset(joint_state.position);
  updated_filters_ = true;
}

void ServoCalcs::composeJointTrajMessage(const sensor_msgs::msg::JointState& joint_state,
                                         trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
  joint_trajectory.header.stamp = rclcpp::Time(0);
  joint_trajectory.header.frame_id = parameters_->planning_frame;
  joint_trajectory.joint_names = joint_state.name;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(parameters_->publish_period);
  if (parameters_->publish_joint_positions)
    point.positions = joint_state.position;
  if (parameters_->publish_joint_velocities)
    point.velocities = joint_state.velocity;
  if (parameters_->publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    std::vector<double> acceleration(num_joints_);
    point.accelerations = acceleration;
  }
  joint_trajectory.points.push_back(point);
}

std::vector<const moveit::core::JointModel*>
ServoCalcs::enforcePositionLimits(sensor_msgs::msg::JointState& joint_state) const
{
  // Halt if we're past a joint margin and joint velocity is moving even farther past
  double joint_angle = 0;
  std::vector<const moveit::core::JointModel*> joints_to_halt;
  for (auto joint : joint_model_group_->getActiveJointModels())
  {
    for (std::size_t c = 0; c < joint_state.name.size(); ++c)
    {
      // Use the most recent robot joint state
      if (joint_state.name[c] == joint->getName())
      {
        joint_angle = joint_state.position.at(c);
        break;
      }
    }

    if (!joint->satisfiesPositionBounds(&joint_angle, -parameters_->joint_limit_margin))
    {
      const std::vector<moveit_msgs::msg::JointLimits>& limits = joint->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (!limits.empty())
      {
        // Check if pending velocity command is moving in the right direction
        auto joint_itr = std::find(joint_state.name.begin(), joint_state.name.end(), joint->getName());
        auto joint_idx = std::distance(joint_state.name.begin(), joint_itr);

        if ((joint_state.velocity.at(joint_idx) < 0 &&
             (joint_angle < (limits[0].min_position + parameters_->joint_limit_margin))) ||
            (joint_state.velocity.at(joint_idx) > 0 &&
             (joint_angle > (limits[0].max_position - parameters_->joint_limit_margin))))
        {
          joints_to_halt.push_back(joint);
        }
      }
    }
  }
  if (!joints_to_halt.empty())
  {
    std::ostringstream joints_names;
    std::transform(joints_to_halt.cbegin(), std::prev(joints_to_halt.cend()),
                   std::ostream_iterator<std::string>(joints_names, ", "),
                   [](const auto& joint) { return joint->getName(); });
    joints_names << joints_to_halt.back()->getName();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
                                node_->get_name()
                                    << " " << joints_names.str() << " close to a position limit. Halting.");
  }
  return joints_to_halt;
}

void ServoCalcs::filteredHalt(trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // Prepare the joint trajectory message to stop the robot
  joint_trajectory.points.clear();
  joint_trajectory.points.emplace_back();

  // Deceleration algorithm:
  // Set positions to original_joint_state_
  // Filter
  // Calculate velocities
  // Check if velocities are close to zero. Round to zero, if so.
  // Set done_stopping_ flag
  assert(original_joint_state_.position.size() >= num_joints_);
  joint_trajectory.points[0].positions = original_joint_state_.position;
  smoother_->doSmoothing(joint_trajectory.points[0].positions);
  done_stopping_ = true;
  if (parameters_->publish_joint_velocities)
  {
    joint_trajectory.points[0].velocities = std::vector<double>(num_joints_, 0);
    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      joint_trajectory.points[0].velocities.at(i) =
          (joint_trajectory.points[0].positions.at(i) - original_joint_state_.position.at(i)) /
          parameters_->publish_period;
      // If velocity is very close to zero, round to zero
      if (joint_trajectory.points[0].velocities.at(i) > STOPPED_VELOCITY_EPS)
      {
        done_stopping_ = false;
      }
    }
    // If every joint is very close to stopped, round velocity to zero
    if (done_stopping_)
    {
      std::fill(joint_trajectory.points[0].velocities.begin(), joint_trajectory.points[0].velocities.end(), 0);
    }
  }

  if (parameters_->publish_joint_accelerations)
  {
    joint_trajectory.points[0].accelerations = std::vector<double>(num_joints_, 0);
    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      joint_trajectory.points[0].accelerations.at(i) =
          (joint_trajectory.points[0].velocities.at(i) - original_joint_state_.velocity.at(i)) /
          parameters_->publish_period;
    }
  }

  joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(parameters_->publish_period);
}

void ServoCalcs::suddenHalt(sensor_msgs::msg::JointState& joint_state,
                            const std::vector<const moveit::core::JointModel*>& joints_to_halt) const
{
  // Set the position to the original position, and velocity to 0 for input joints
  for (const auto& joint_to_halt : joints_to_halt)
  {
    const auto joint_it = std::find(joint_state.name.cbegin(), joint_state.name.cend(), joint_to_halt->getName());
    if (joint_it != joint_state.name.cend())
    {
      const auto joint_index = std::distance(joint_state.name.cbegin(), joint_it);
      joint_state.position.at(joint_index) = original_joint_state_.position.at(joint_index);
      joint_state.velocity.at(joint_index) = 0.0;
    }
  }
}

void ServoCalcs::updateJoints()
{
  // Get the latest joint group positions
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->copyJointGroupPositions(joint_model_group_, internal_joint_state_.position);
  current_state_->copyJointGroupVelocities(joint_model_group_, internal_joint_state_.velocity);

  // Cache the original joints in case they need to be reset
  original_joint_state_ = internal_joint_state_;
}

bool ServoCalcs::checkValidCommand(const control_msgs::msg::JointJog& cmd)
{
  for (double velocity : cmd.velocities)
  {
    if (std::isnan(velocity))
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "nan in incoming command. Skipping this datapoint.");
      return false;
    }
  }
  return true;
}

bool ServoCalcs::checkValidCommand(const geometry_msgs::msg::TwistStamped& cmd)
{
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_->command_in_type == "unitless")
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                  "Component of incoming command is >1. Skipping this datapoint.");
      return false;
    }
  }

  return true;
}

// Scale the incoming jog command. Returns a vector of position deltas
Eigen::VectorXd ServoCalcs::scaleCartesianCommand(const geometry_msgs::msg::TwistStamped& command)
{
  Eigen::VectorXd result(6);
  result.setZero();  // Or the else case below leads to misery

  // Apply user-defined scaling if inputs are unitless [-1:1]
  if (parameters_->command_in_type == "unitless")
  {
    result[0] = parameters_->linear_scale * parameters_->publish_period * command.twist.linear.x;
    result[1] = parameters_->linear_scale * parameters_->publish_period * command.twist.linear.y;
    result[2] = parameters_->linear_scale * parameters_->publish_period * command.twist.linear.z;
    result[3] = parameters_->rotational_scale * parameters_->publish_period * command.twist.angular.x;
    result[4] = parameters_->rotational_scale * parameters_->publish_period * command.twist.angular.y;
    result[5] = parameters_->rotational_scale * parameters_->publish_period * command.twist.angular.z;
  }
  // Otherwise, commands are in m/s and rad/s
  else if (parameters_->command_in_type == "speed_units")
  {
    result[0] = command.twist.linear.x * parameters_->publish_period;
    result[1] = command.twist.linear.y * parameters_->publish_period;
    result[2] = command.twist.linear.z * parameters_->publish_period;
    result[3] = command.twist.angular.x * parameters_->publish_period;
    result[4] = command.twist.angular.y * parameters_->publish_period;
    result[5] = command.twist.angular.z * parameters_->publish_period;
  }
  else
  {
    rclcpp::Clock& clock = *node_->get_clock();
    RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "Unexpected command_in_type");
  }

  return result;
}

Eigen::VectorXd ServoCalcs::scaleJointCommand(const control_msgs::msg::JointJog& command)
{
  Eigen::VectorXd result(num_joints_);
  result.setZero();

  std::size_t c;
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    try
    {
      c = joint_state_name_map_.at(command.joint_names[m]);
    }
    catch (const std::out_of_range& e)
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD, "Ignoring joint " << command.joint_names[m]);
      continue;
    }
    // Apply user-defined scaling if inputs are unitless [-1:1]
    // 大概明白了 我们应该只能发送速度
    // 然后他根据这个速度乘以 parameters_->publish_period 得到最终的关节角
    if (parameters_->command_in_type == "unitless")
      // unitless 应该是归一化速度 -1 就是全速反方向
      result[c] = command.velocities[m] * parameters_->joint_scale * parameters_->publish_period;
    // Otherwise, commands are in m/s and rad/s
    else if (parameters_->command_in_type == "speed_units")
      // speed_units 就是发送明确的关节速度
      result[c] = command.velocities[m] * parameters_->publish_period;
    else
    {
      rclcpp::Clock& clock = *node_->get_clock();
      RCLCPP_ERROR_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
                                   "Unexpected command_in_type, check yaml file.");
    }
    // // 添加debug
    // {
    //   // 到这个地方是正常的 发送一个关节就只有一个关节
    //   rclcpp::Clock& clock = *node_->get_clock();
    //   RCLCPP_WARN_STREAM_THROTTLE(LOGGER, clock, ROS_LOG_THROTTLE_PERIOD,
    //                               "scaleJointCommand() change joint " << command.joint_names[m]);
    // }
  }

  return result;
}

void ServoCalcs::removeDimension(Eigen::MatrixXd& jacobian, Eigen::VectorXd& delta_x, unsigned int row_to_remove) const
{
  unsigned int num_rows = jacobian.rows() - 1;
  unsigned int num_cols = jacobian.cols();

  if (row_to_remove < num_rows)
  {
    jacobian.block(row_to_remove, 0, num_rows - row_to_remove, num_cols) =
        jacobian.block(row_to_remove + 1, 0, num_rows - row_to_remove, num_cols);
    delta_x.segment(row_to_remove, num_rows - row_to_remove) =
        delta_x.segment(row_to_remove + 1, num_rows - row_to_remove);
  }
  jacobian.conservativeResize(num_rows, num_cols);
  delta_x.conservativeResize(num_rows);
}

void ServoCalcs::removeDriftDimensions(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x)
{
  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True in the vector drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
  for (auto dimension = matrix.rows() - 1; dimension >= 0; --dimension)
  {
    if (drift_dimensions_[dimension] && matrix.rows() > 1)
    {
      removeDimension(matrix, delta_x, dimension);
    }
  }
}

void ServoCalcs::enforceControlDimensions(geometry_msgs::msg::TwistStamped& command)
{
  // Can't loop through the message, so check them all
  if (!control_dimensions_[0])
    command.twist.linear.x = 0;
  if (!control_dimensions_[1])
    command.twist.linear.y = 0;
  if (!control_dimensions_[2])
    command.twist.linear.z = 0;
  if (!control_dimensions_[3])
    command.twist.angular.x = 0;
  if (!control_dimensions_[4])
    command.twist.angular.y = 0;
  if (!control_dimensions_[5])
    command.twist.angular.z = 0;
}

bool ServoCalcs::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  transform = tf_moveit_to_robot_cmd_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_robot_cmd_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform =
      convertIsometryToTransform(tf_moveit_to_robot_cmd_frame_, parameters_->planning_frame, robot_link_command_frame_);
  return true;
}

bool ServoCalcs::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  transform = tf_moveit_to_ee_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getEEFrameTransform(geometry_msgs::msg::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_ee_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform =
      convertIsometryToTransform(tf_moveit_to_ee_frame_, parameters_->planning_frame, parameters_->ee_frame_name);
  return true;
}

void ServoCalcs::twistStampedCB(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  latest_twist_stamped_ = msg;
  latest_twist_cmd_is_nonzero_ = isNonZero(*latest_twist_stamped_);

  if (msg->header.stamp != rclcpp::Time(0.))
    latest_twist_command_stamp_ = msg->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::jointCmdCB(const control_msgs::msg::JointJog::ConstSharedPtr& msg)
{
  const std::lock_guard<std::mutex> lock(main_loop_mutex_);
  latest_joint_cmd_ = msg;
  latest_joint_cmd_is_nonzero_ = isNonZero(*latest_joint_cmd_);

  if (msg->header.stamp != rclcpp::Time(0.))
    latest_joint_command_stamp_ = msg->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::collisionVelocityScaleCB(const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
  collision_velocity_scale_ = msg->data;
}

void ServoCalcs::changeDriftDimensions(const std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Request>& req,
                                       const std::shared_ptr<moveit_msgs::srv::ChangeDriftDimensions::Response>& res)
{
  drift_dimensions_[0] = req->drift_x_translation;
  drift_dimensions_[1] = req->drift_y_translation;
  drift_dimensions_[2] = req->drift_z_translation;
  drift_dimensions_[3] = req->drift_x_rotation;
  drift_dimensions_[4] = req->drift_y_rotation;
  drift_dimensions_[5] = req->drift_z_rotation;

  res->success = true;
}

void ServoCalcs::changeControlDimensions(const std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Request>& req,
                                         const std::shared_ptr<moveit_msgs::srv::ChangeControlDimensions::Response>& res)
{
  control_dimensions_[0] = req->control_x_translation;
  control_dimensions_[1] = req->control_y_translation;
  control_dimensions_[2] = req->control_z_translation;
  control_dimensions_[3] = req->control_x_rotation;
  control_dimensions_[4] = req->control_y_rotation;
  control_dimensions_[5] = req->control_z_rotation;

  res->success = true;
}

bool ServoCalcs::resetServoStatus(const std::shared_ptr<std_srvs::srv::Empty::Request>& /*req*/,
                                  const std::shared_ptr<std_srvs::srv::Empty::Response>& /*res*/)
{
  status_ = StatusCode::NO_WARNING;
  return true;
}

void ServoCalcs::setPaused(bool paused)
{
  paused_ = paused;
}

}  // namespace moveit_servo

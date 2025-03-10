/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Dave Coleman */

#pragma once

#include <moveit/macros/class_forward.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit_ros_planning/kinematics_parameters.hpp>

namespace kinematics_plugin_loader
{
MOVEIT_CLASS_FORWARD(KinematicsPluginLoader);  // Defines KinematicsPluginLoaderPtr, ConstPtr, WeakPtr... etc

/** \brief Helper class for loading kinematics solvers */
class KinematicsPluginLoader
{
public:
  /** \brief Load the kinematics solvers based on information on the
      ROS parameter server. Take node as an argument and as optional argument the name of the
      ROS parameter under which the robot description can be
      found. This is passed to the kinematics solver initialization as
      well as used to read the SRDF document when needed. */
  KinematicsPluginLoader(const rclcpp::Node::SharedPtr& node,
                         const std::string& robot_description = "robot_description")
    : node_(node)
    , robot_description_(robot_description)
    , logger_(moveit::getLogger("moveit.ros.kinematics_plugin_loader"))
  {
  }

  /** \brief Get a function pointer that allocates and initializes a kinematics solver. If not previously called, this
   * function reads ROS parameters for the groups defined in the SRDF. */
  moveit::core::SolverAllocatorFn getLoaderFunction(const srdf::ModelSharedPtr& srdf_model);

  /** \brief Get the groups for which the function pointer returned by getLoaderFunction() can allocate a solver */
  const std::vector<std::string>& getKnownGroups() const
  {
    return groups_;
  }

  /** \brief Get a map from group name to default IK timeout */
  const std::map<std::string, double>& getIKTimeout() const
  {
    return ik_timeout_;
  }

  void status() const;

private:
  const rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, std::shared_ptr<kinematics::ParamListener>> group_param_listener_;
  std::unordered_map<std::string, kinematics::Params> group_params_;
  std::string robot_description_;

  MOVEIT_CLASS_FORWARD(KinematicsLoaderImpl);  // Defines KinematicsLoaderImplPtr, ConstPtr, WeakPtr... etc
  KinematicsLoaderImplPtr loader_;

  std::vector<std::string> groups_;
  std::map<std::string, double> ik_timeout_;
  rclcpp::Logger logger_;
};
}  // namespace kinematics_plugin_loader

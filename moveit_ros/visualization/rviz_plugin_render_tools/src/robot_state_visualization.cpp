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

/* Author: Ioan Sucan */

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.hpp>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.hpp>
#include <moveit/rviz_plugin_render_tools/render_shapes.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <QApplication>
#include <moveit/utils/logger.hpp>

namespace moveit_rviz_plugin
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.rviz_plugin_render_tools.robot_state_visualization");
}
}  // namespace

RobotStateVisualization::RobotStateVisualization(Ogre::SceneNode* root_node, rviz_common::DisplayContext* context,
                                                 const std::string& name,
                                                 rviz_common::properties::Property* parent_property)
  : robot_(root_node, context, name, parent_property)
  , octree_voxel_render_mode_(OCTOMAP_OCCUPIED_VOXELS)
  , octree_voxel_color_mode_(OCTOMAP_Z_AXIS_COLOR)
  , visible_(true)
  , visual_visible_(true)
  , collision_visible_(false)
{
  default_attached_object_color_.r = 0.0f;
  default_attached_object_color_.g = 0.7f;
  default_attached_object_color_.b = 0.0f;
  default_attached_object_color_.a = 1.0f;
  render_shapes_ = std::make_shared<RenderShapes>(context);
}

void RobotStateVisualization::load(const urdf::ModelInterface& descr, bool visual, bool collision)
{
  // clear previously loaded model
  clear();

  robot_.load(descr, visual, collision);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
}

void RobotStateVisualization::clear()
{
  render_shapes_->clear();
  robot_.clear();
}

void RobotStateVisualization::setDefaultAttachedObjectColor(const std_msgs::msg::ColorRGBA& default_attached_object_color)
{
  default_attached_object_color_ = default_attached_object_color;
}

void RobotStateVisualization::updateAttachedObjectColors(const std_msgs::msg::ColorRGBA& attached_object_color)
{
  render_shapes_->updateShapeColors(attached_object_color.r, attached_object_color.g, attached_object_color.b,
                                    robot_.getAlpha());
}

void RobotStateVisualization::update(const moveit::core::RobotStateConstPtr& robot_state)
{
  updateHelper(robot_state, default_attached_object_color_, nullptr);
}

void RobotStateVisualization::update(const moveit::core::RobotStateConstPtr& robot_state,
                                     const std_msgs::msg::ColorRGBA& default_attached_object_color)
{
  updateHelper(robot_state, default_attached_object_color, nullptr);
}

void RobotStateVisualization::update(const moveit::core::RobotStateConstPtr& robot_state,
                                     const std_msgs::msg::ColorRGBA& default_attached_object_color,
                                     const std::map<std::string, std_msgs::msg::ColorRGBA>& color_map)
{
  updateHelper(robot_state, default_attached_object_color, &color_map);
}

void RobotStateVisualization::updateHelper(const moveit::core::RobotStateConstPtr& robot_state,
                                           const std_msgs::msg::ColorRGBA& default_attached_object_color,
                                           const std::map<std::string, std_msgs::msg::ColorRGBA>* color_map)
{
  robot_.update(PlanningLinkUpdater(robot_state));
  render_shapes_->clear();

  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  robot_state->getAttachedBodies(attached_bodies);
  for (const moveit::core::AttachedBody* attached_body : attached_bodies)
  {
    std_msgs::msg::ColorRGBA color = default_attached_object_color;
    double alpha = robot_.getAlpha();
    if (color_map)
    {
      std::map<std::string, std_msgs::msg::ColorRGBA>::const_iterator it = color_map->find(attached_body->getName());
      if (it != color_map->end())
      {
        color = it->second;
        alpha = color.a;
      }
    }
    rviz_default_plugins::robot::RobotLink* link = robot_.getLink(attached_body->getAttachedLinkName());
    if (!link)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Link " << attached_body->getAttachedLinkName() << " not found in rviz::Robot");
      continue;
    }
    Ogre::ColourValue rcolor(color.r, color.g, color.b, color.a);
    const EigenSTL::vector_Isometry3d& ab_t = attached_body->getShapePosesInLinkFrame();
    const std::vector<shapes::ShapeConstPtr>& ab_shapes = attached_body->getShapes();
    for (std::size_t j = 0; j < ab_shapes.size(); ++j)
    {
      render_shapes_->renderShape(link->getVisualNode(), ab_shapes[j].get(), ab_t[j], octree_voxel_render_mode_,
                                  octree_voxel_color_mode_, rcolor, alpha);
      render_shapes_->renderShape(link->getCollisionNode(), ab_shapes[j].get(), ab_t[j], octree_voxel_render_mode_,
                                  octree_voxel_color_mode_, rcolor, alpha);
    }
  }
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
}

void RobotStateVisualization::updateKinematicState(const moveit::core::RobotStateConstPtr& robot_state)
{
  robot_.update(PlanningLinkUpdater(robot_state));
}

void RobotStateVisualization::setVisible(bool visible)
{
  visible_ = visible;
  robot_.setVisible(visible);
}

void RobotStateVisualization::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  robot_.setVisualVisible(visible);
}

void RobotStateVisualization::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  robot_.setCollisionVisible(visible);
}

void RobotStateVisualization::setAlpha(double alpha)
{
  robot_.setAlpha(alpha);
}
}  // namespace moveit_rviz_plugin

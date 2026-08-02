// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /dock_robot action to dock the robot.
///
/// Input ports:
///   dock_id   (string) – named dock instance (e.g. "home_dock")
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class DockRobot : public BT::StatefulActionNode
{
public:
  using DockAction = nav2_msgs::action::DockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<DockAction>;
  using NavAction = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  DockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id", "home_dock", "Named dock instance"),
            BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// kPrestage: NavigateToPose to the pre-staging waypoint on the dock axis
  /// (ctx->dock_prestage_distance_m behind the dock, facing it) so the
  /// transit's terminal in-place pivot happens away from the dock's curb
  /// stones. kDock: the /dock_robot action itself (its internal
  /// nav-to-staging leg is then a short straight along the axis).
  enum class Phase
  {
    kPrestage,
    kDock
  };

  /// Send the /dock_robot goal (phase kDock). Returns false if the action
  /// server vanished (caller must fail the node).
  bool sendDockGoal(const std::shared_ptr<BTContext>& ctx);

  Phase phase_{Phase::kDock};
  std::string dock_id_{"home_dock"};
  std::string dock_type_{"simple_charging_dock"};
  rclcpp_action::Client<DockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_goal_future_;
  NavGoalHandle::SharedPtr nav_goal_handle_;
};

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /undock_robot action to undock the robot.
///
/// Input ports:
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class UndockRobot : public BT::StatefulActionNode
{
public:
  using UndockAction = nav2_msgs::action::UndockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<UndockAction>;

  UndockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<UndockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------------------
// RecordResumeUndockFailure
// ---------------------------------------------------------------------------

/// Increments the resume_undock_failures counter in BTContext.
/// Always returns SUCCESS so it can be placed inside any sequence.
class RecordResumeUndockFailure : public BT::SyncActionNode
{
public:
  RecordResumeUndockFailure(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

}  // namespace mowgli_behavior

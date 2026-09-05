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

#include "mowgli_behavior/docking_nodes.hpp"

#include <cmath>

#include "action_msgs/msg/goal_status.hpp"
#include "mowgli_behavior/dock_alignment.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

void DockRobot::log_contact_delta(const std::shared_ptr<BTContext>& ctx, uint16_t num_retries) const
{
  const auto d =
      ComputeDockContactDelta(ctx->gps_x, ctx->gps_y, ctx->dock_x, ctx->dock_y, ctx->dock_yaw);

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockRobot: contact claimed (retry %u) at (%.3f, %.3f) vs dock "
              "(%.3f, %.3f, %.2f°) — along=%+.3f m cross=%+.3f m range=%.3f m",
              num_retries,
              ctx->gps_x,
              ctx->gps_y,
              ctx->dock_x,
              ctx->dock_y,
              ctx->dock_yaw * 180.0 / M_PI,
              d.along_m,
              d.cross_m,
              d.range_m);
}

bool DockRobot::sendDockGoal(const std::shared_ptr<BTContext>& ctx)
{
  DockAction::Goal goal_msg;
  goal_msg.dock_id = dock_id_;
  goal_msg.dock_type = dock_type_;
  goal_msg.navigate_to_staging_pose = true;

  last_feedback_state_ = DockAction::Feedback::NONE;

  auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions{};
  send_goal_options.feedback_callback =
      [this, ctx](GoalHandle::SharedPtr, const std::shared_ptr<const DockAction::Feedback> fb)
  {
    // Log once on each ENTRY into WAIT_FOR_CHARGE — the server re-enters it on
    // every retry, and each entry is a separate claimed contact worth recording.
    const uint16_t prev = last_feedback_state_.exchange(fb->state);
    if (fb->state == DockAction::Feedback::WAIT_FOR_CHARGE &&
        prev != DockAction::Feedback::WAIT_FOR_CHARGE)
    {
      log_contact_delta(ctx, fb->num_retries);
    }
  };
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();
  phase_ = Phase::kDock;

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockRobot: goal sent (dock_id='%s', dock_type='%s')",
              dock_id_.c_str(),
              dock_type_.c_str());
  return true;
}

BT::NodeStatus DockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  dock_id_ = "home_dock";
  if (auto res = getInput<std::string>("dock_id"))
  {
    dock_id_ = res.value();
  }

  dock_type_ = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type_ = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<DockAction>(ctx->node, "/dock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: /dock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Pre-staging waypoint (dock_prestage_distance_m > 0): navigate to a pose
  // ON the dock approach axis, prestage-distance behind the dock and FACING
  // it, before handing over to /dock_robot. The transit's terminal in-place
  // pivot (RotationShim rotate_to_goal_heading — up to 90°+ when the path
  // arrives cross-axis) then happens at this waypoint instead of at the
  // staging pose 1.5 m in front of the dock, where the rear-axle pivot's
  // nose sweep was clipping the curb stones (field 2026-08-02). The
  // remaining nav-to-staging leg inside /dock_robot becomes a short straight
  // along the axis with a near-zero terminal rotation. The zero-pose guard
  // covers a fresh install whose dock pose was never calibrated.
  phase_ = Phase::kDock;
  const double prestage_d = ctx->dock_prestage_distance_m;
  const bool dock_pose_known = !(ctx->dock_x == 0.0 && ctx->dock_y == 0.0 && ctx->dock_yaw == 0.0);
  if (prestage_d > 0.0 && dock_pose_known)
  {
    if (!nav_client_)
    {
      nav_client_ = rclcpp_action::create_client<NavAction>(ctx->node, "/navigate_to_pose");
    }
    if (nav_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      NavAction::Goal nav_goal;
      nav_goal.pose.header.frame_id = "map";
      nav_goal.pose.header.stamp = ctx->node->now();
      nav_goal.pose.pose.position.x = ctx->dock_x - prestage_d * std::cos(ctx->dock_yaw);
      nav_goal.pose.pose.position.y = ctx->dock_y - prestage_d * std::sin(ctx->dock_yaw);
      nav_goal.pose.pose.orientation.z = std::sin(ctx->dock_yaw * 0.5);
      nav_goal.pose.pose.orientation.w = std::cos(ctx->dock_yaw * 0.5);

      auto nav_options = rclcpp_action::Client<NavAction>::SendGoalOptions{};
      nav_goal_future_ = nav_client_->async_send_goal(nav_goal, nav_options);
      nav_goal_handle_.reset();
      phase_ = Phase::kPrestage;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "DockRobot: pre-staging %.1f m behind the dock at (%.2f, %.2f) — terminal "
                  "pivot happens there, not at the staging pose",
                  prestage_d,
                  nav_goal.pose.pose.position.x,
                  nav_goal.pose.pose.position.y);
    }
    else
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "DockRobot: /navigate_to_pose unavailable — skipping pre-staging");
    }
  }

  if (phase_ == Phase::kDock)
  {
    sendDockGoal(ctx);
  }

  // Mark the blade-off dock transit active ONLY on the RUNNING path (the
  // action-server-unavailable early return above leaves this false). Read by
  // IsDocking so BoundaryGuard exempts this transit — the pre-staging leg
  // needs the exemption just as much as /dock_robot's own staging leg (the
  // dock corridor lies outside the mowing polygon). See bt_context.hpp.
  ctx->docking_active = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (phase_ == Phase::kPrestage)
  {
    if (!nav_goal_handle_)
    {
      if (nav_goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      nav_goal_handle_ = nav_goal_future_.get();
      if (!nav_goal_handle_)
      {
        // Pre-staging is an optimisation, never a blocker: fall through to
        // /dock_robot, whose internal nav-to-staging still gets the robot
        // there (with the old terminal pivot at the staging pose).
        RCLCPP_WARN(ctx->node->get_logger(),
                    "DockRobot: pre-staging goal rejected — docking directly");
        sendDockGoal(ctx);
        return BT::NodeStatus::RUNNING;
      }
    }
    switch (nav_goal_handle_->get_status())
    {
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        RCLCPP_INFO(ctx->node->get_logger(),
                    "DockRobot: pre-staging waypoint reached — starting dock approach");
        nav_goal_handle_.reset();
        sendDockGoal(ctx);
        return BT::NodeStatus::RUNNING;
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        RCLCPP_WARN(ctx->node->get_logger(),
                    "DockRobot: pre-staging navigation failed — docking directly");
        nav_goal_handle_.reset();
        sendDockGoal(ctx);
        return BT::NodeStatus::RUNNING;
      default:
        return BT::NodeStatus::RUNNING;
    }
  }

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "DockRobot: goal was rejected by the action server");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      ctx->docking_active = false;
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking aborted");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking canceled");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockRobot::onHalted()
{
  // Clear the dock-transit flag UNCONDITIONALLY (even if goal_handle_ was not
  // yet confirmed): BehaviorTree.CPP invokes onHalted() whenever a RUNNING
  // DockRobot is halted by a parent (e.g. a new operator command, or the root
  // ReactiveSequence re-priority), and the flag must never survive that halt —
  // otherwise BoundaryGuard would stay exempted after the transit ended.
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->docking_active = false;
  if (nav_goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: canceling pre-staging navigation");
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_.reset();
  }
  if (goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus UndockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<UndockAction>(ctx->node, "/undock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: /undock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  UndockAction::Goal goal_msg;
  goal_msg.dock_type = dock_type;

  auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "UndockRobot: goal sent (dock_type='%s')",
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UndockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "UndockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: undocking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void UndockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// RecordResumeUndockFailure
// ---------------------------------------------------------------------------

BT::NodeStatus RecordResumeUndockFailure::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->resume_undock_failures++;
  RCLCPP_WARN(ctx->node->get_logger(),
              "RecordResumeUndockFailure: resume undock failures = %d",
              ctx->resume_undock_failures);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior

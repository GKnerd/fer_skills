#include "fer_skills/skill_server_node.hpp"

#include <thread>

namespace fer_skills
{

namespace
{
constexpr const char * kReadyState = "ready";
}

rclcpp_action::GoalResponse SkillServerNode::handle_go_home_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GoHome::Goal> /*goal*/)
{
  RCLCPP_INFO(node_->get_logger(), "GoHome: goal received");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillServerNode::handle_go_home_cancel(
  const std::shared_ptr<GoalHandleGoHome> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "GoHome: cancel requested");
  arm_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillServerNode::handle_go_home_accepted(const std::shared_ptr<GoalHandleGoHome> goal_handle)
{
  std::thread{[this, goal_handle]() { execute_go_home(goal_handle); }}.detach();
}

void SkillServerNode::execute_go_home(const std::shared_ptr<GoalHandleGoHome> goal_handle)
{
  auto feedback = std::make_shared<GoHome::Feedback>();
  auto result = std::make_shared<GoHome::Result>();

  feedback->status = "planning";
  goal_handle->publish_feedback(feedback);

  arm_->setStartStateToCurrentState();
  arm_->setNamedTarget(kReadyState);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_ok =
    (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!plan_ok) {
    result->success = false;
    result->message = "PLANNING_FAILED";
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "GoHome: planning failed");
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "CANCELLED";
    goal_handle->canceled(result);
    return;
  }

  feedback->status = "executing";
  goal_handle->publish_feedback(feedback);

  const auto exec_code = arm_->execute(plan);
  const bool exec_ok = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "CANCELLED";
    goal_handle->canceled(result);
    return;
  }

  if (!exec_ok) {
    result->success = false;
    result->message = "EXECUTION_FAILED";
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "GoHome: execution failed");
    return;
  }

  result->success = true;
  result->message = "OK";
  goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "GoHome: succeeded");
}

}  // namespace fer_skills

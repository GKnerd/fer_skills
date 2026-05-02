#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <thread>

#include "fer_skills/skill_server_node.hpp"



namespace fer_skills
{

rclcpp_action::GoalResponse SkillServerNode::handle_move_to_pose_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "MoveToPose: goal received (frame=%s, x=%.3f y=%.3f z=%.3f)",
    goal->target_pose.header.frame_id.c_str(),
    goal->target_pose.pose.position.x,
    goal->target_pose.pose.position.y,
    goal->target_pose.pose.position.z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillServerNode::handle_move_to_pose_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> /*goal_handle*/)
{
  arm_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillServerNode::handle_move_to_pose_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  std::thread{[this, goal_handle]() { execute_move_to_pose(goal_handle); }}.detach();
}

// TO-DO: split this into two functions (one planning and one executing function)
// TO-DO: make the edge case handling more generic. too much repeated code right now. 
/*
TO-DO: we have defined setPoseTarget() this is restrictive to the planner as it requires full 6D pose 
to solve the problem. 
Option 1: setPositionTarget() is a viable alternative but might result in a lot of failures in planning.
Option 2: Constrain the approach axis. Free the wrist roll (aka TCP points down but rotation about z axis is free.)
Const
*/ 
void SkillServerNode::execute_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  auto result = std::make_shared<MoveToPose::Result>();
  
  feedback->status = "planning";
  goal_handle->publish_feedback(feedback);

  const auto goal = goal_handle -> get_goal();
  
  arm_->setStartStateToCurrentState();
  arm_->setPoseTarget(goal -> target_pose);
  arm_->setMaxVelocityScalingFactor(goal->velocity_scaling);
  arm_->setMaxAccelerationScalingFactor(goal->acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_ok {
    (arm_->plan(plan)) == moveit::core::MoveItErrorCode::SUCCESS
  };
  // Planning failed
  if (!plan_ok)
  {
    result->success = false;
    result->message = "PLANNING FAILED";
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "MoveToPose: planning failed");
    return;
  }

  // Planning cancelled
  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "CANCELLED";
    goal_handle->canceled(result);
    return;
  }

  feedback->status = "Executing";
  //TO-DO: How to implement progress message
  // feedback->progress =... ;
  goal_handle->publish_feedback(feedback);

  const auto exec_plan = arm_->execute(plan);
  const bool exec_ok = (exec_plan == moveit::core::MoveItErrorCode::SUCCESS);

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
    RCLCPP_ERROR(node_->get_logger(), "MoveToPose: execution failed");
    return;
  }

  result->success = true;
  result->message = "OK";
  goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "MoveToPose: succeeded");
}

}  // namespace fer_skills

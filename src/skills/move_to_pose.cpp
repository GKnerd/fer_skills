#include "fer_skills/skill_server_node.hpp"

#include <thread>

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

void SkillServerNode::execute_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  // TODO: implement.
  // Plan: setStartStateToCurrentState() -> setPoseTarget(goal->target_pose)
  // -> setMaxVelocityScalingFactor / setMaxAccelerationScalingFactor
  // -> plan() -> execute().
  auto result = std::make_shared<MoveToPose::Result>();
  result->success = false;
  result->message = "NOT_IMPLEMENTED";
  goal_handle->abort(result);
}

}  // namespace fer_skills

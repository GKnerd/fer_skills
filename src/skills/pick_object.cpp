#include "fer_skills/skill_server_node.hpp"

namespace fer_skills
{

rclcpp_action::GoalResponse SkillServerNode::handle_pick_object_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PickObject::Goal> goal)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "PickObject: goal received (x=%.3f y=%.3f z=%.3f, approach=%.3f)",
    goal->object_position.point.x,
    goal->object_position.point.y,
    goal->object_position.point.z,
    goal->approach_height);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillServerNode::handle_pick_object_cancel(
  const std::shared_ptr<GoalHandlePickObject> /*goal_handle*/)
{
  arm_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillServerNode::handle_pick_object_accepted(
  const std::shared_ptr<GoalHandlePickObject> goal_handle)
{
  std::thread{[this, goal_handle]() { execute_pick_object(goal_handle); }}.detach();
}

void SkillServerNode::execute_pick_object(
  const std::shared_ptr<GoalHandlePickObject> goal_handle)
{
  // TODO: implement using MoveIt Task Constructor.
  // Stages: GenerateGraspPose (top-down) -> Connect -> MoveRelative (approach down)
  //         -> close gripper via control_msgs/action/GripperCommand
  //         -> MoveRelative (lift up).
  auto result = std::make_shared<PickObject::Result>();
  result->success = false;
  result->message = "NOT_IMPLEMENTED";
  goal_handle->abort(result);
}
}  // namespace fer_skills

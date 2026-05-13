#include <fer_skills/skill_server_node.hpp>
#include <memory>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/types.hpp>
#include <thread>


namespace fer_skills
{

rclcpp_action::GoalResponse SkillServerNode::handle_control_gripper_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ControlGripper::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "Control Gripper: goal received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillServerNode::handle_control_gripper_cancel(
    const std::shared_ptr<GoalHandleControlGripper> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Control Gripper: cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;

}

void SkillServerNode::handle_control_gripper_accepted(
    const std::shared_ptr<GoalHandleControlGripper> goal_handle)
{
    std::thread{[this, goal_handle]() { execute_control_gripper(goal_handle); }}.detach();
}

void SkillServerNode::execute_control_gripper(
    const std::shared_ptr<GoalHandleControlGripper> goal_handle
)
{
    auto feedback = std::make_shared<ControlGripper::Feedback>();
    auto result = std::make_shared<ControlGripper::Result>();
    const std::string &state = "ready";

    feedback->status = "planning";
    goal_handle->publish_feedback(feedback);

    hand_->setStartStateToCurrentState();
    hand_->setNamedTarget(state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_ok =
        (hand_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok) {
        result->success = false;
        result->message = "PLANNING_FAILED";
        goal_handle->abort(result);
        RCLCPP_ERROR(node_->get_logger(), "ControlGripper: planning failed");
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

    const auto exec_code = hand_->execute(plan);
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
        RCLCPP_ERROR(node_->get_logger(), "ControlGripper: execution failed");
        return;
    }

    result->success = true;
    result->message = "OK";
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "ControlGripper: succeeded");
}

} // namespace fer_skills

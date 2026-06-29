#include "fer_skills/skill_server_node.hpp"
#include "fer_skills/mtc_place_object.hpp"

#include <algorithm>
#include <memory>
#include <thread>

#include <Eigen/Geometry>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>


namespace fer_skills
{

rclcpp_action::GoalResponse SkillServerNode::handle_place_object_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlaceObject::Goal> goal)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "PlaceObject: goal received (x=%.3f y=%.3f z=%.3f, approach=%.3f)",
    goal->place_position.point.x,
    goal->place_position.point.y,
    goal->place_position.point.z,
    goal->approach_height);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillServerNode::handle_place_object_cancel(
  const std::shared_ptr<GoalHandlePlaceObject> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "PlaceObject: cancel requested");
  arm_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillServerNode::handle_place_object_accepted(
  const std::shared_ptr<GoalHandlePlaceObject> goal_handle)
{
  std::thread{[this, goal_handle]() { execute_place_object(goal_handle); }}.detach();
}

void SkillServerNode::execute_place_object(
  const std::shared_ptr<GoalHandlePlaceObject> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PlaceObject::Feedback>();
  auto result   = std::make_shared<PlaceObject::Result>();

  // ---- Translate goal → PlaceConfig ----
  //
  // TODO: like PickObject, the action currently carries a Cartesian point, not
  // an object id. The MTC pipeline references objects by id (string). For now
  // we assume the object attached during the preceding pick has this hardcoded
  // id; extend PlaceObject.action with `string object_id` and pull it from the
  // goal instead.
  place_object::PlaceConfig config;
  config.object_id = "cylinder_1";
  // Tie the MTC task name to this action goal's UUID for end-to-end traceability.
  config.task_id = rclcpp_action::to_string(goal_handle->get_goal_id());

  // ---- DIAGNOSTIC: the object must be ATTACHED to the hand before placing ----
  // Unlike pick (which expects the object as a world object), place expects it
  // attached to the gripper — that attachment is what GeneratePlacePose needs
  // to locate the object and compute the place IK.
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto attached = psi.getAttachedObjects({config.object_id});

    if (attached.find(config.object_id) == attached.end()) {
      result->success = false;
      result->message = "OBJECT_NOT_ATTACHED: '" + config.object_id +
                        "' is not attached to the hand; nothing to place";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(node_->get_logger(),
                "PlaceObject diag: '%s' is attached; proceeding to place",
                config.object_id.c_str());
  }

  // Place pose: where the OBJECT's frame should end up. The place IK frame is
  // the object itself (see PlaceObject::make_place_pose), so this is simply the
  // object's desired pose in the goal frame — no TCP/grasp transform is needed
  // here, unlike pick. We keep the object upright (identity orientation) for
  // now; this Eigen hook is where a real placement orientation will go.
  {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    const Eigen::Quaterniond q(t.linear());

    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = goal->place_position.header.frame_id;
    place_pose.pose.position = goal->place_position.point;
    place_pose.pose.orientation.x = q.x();
    place_pose.pose.orientation.y = q.y();
    place_pose.pose.orientation.z = q.z();
    place_pose.pose.orientation.w = q.w();
    config.place_pose = place_pose;
  }

  // Retreat: lift the (now empty) hand straight up, scaled by approach_height.
  config.retreat_direction.header.frame_id = "base";
  config.retreat_direction.vector.z = 1.0;
  config.retreat_min_distance = std::max(0.05f, goal->approach_height * 0.5f);
  config.retreat_max_distance = std::max(0.10f, goal->approach_height);

  // ---- Plan ----
  feedback->current_phase = "planning";
  feedback->progress = 0.1f;
  goal_handle->publish_feedback(feedback);

  const auto plan_result = placer_->plan_place(config);
  if (plan_result.status != place_object::PlanResult::Status::Success) {
    result->success = false;
    result->message = "PLANNING_FAILED: " + plan_result.failure_reason;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "PlaceObject: planning failed");
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "CANCELLED";
    goal_handle->canceled(result);
    return;
  }

  // ---- Execute ----
  feedback->current_phase = "executing";
  feedback->progress = 0.5f;
  goal_handle->publish_feedback(feedback);

  const auto exec_result = placer_->execute_place();

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "CANCELLED";
    goal_handle->canceled(result);
    return;
  }

  if (exec_result.status != place_object::ExecuteResult::Status::Success) {
    result->success = false;
    result->message = "EXECUTION_FAILED: " + exec_result.failure_reason;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "PlaceObject: execution failed");
    return;
  }

  feedback->current_phase = "done";
  feedback->progress = 1.0f;
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "OK";
  goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "PlaceObject: succeeded");
}

}  // namespace fer_skills

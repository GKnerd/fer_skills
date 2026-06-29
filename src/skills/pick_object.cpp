#include "fer_skills/skill_server_node.hpp"
#include "fer_skills/mtc_pick_object.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

#include <Eigen/Geometry>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>


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
  RCLCPP_INFO(node_->get_logger(), "PickObject: cancel requested");
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
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PickObject::Feedback>();
  auto result   = std::make_shared<PickObject::Result>();

  // ---- Translate goal → PickConfig ----
  //
  // TODO: the action message currently carries a Cartesian point, not an
  // object id. The MTC pipeline references objects by id (string). For now we
  // assume the scene loader has registered an object with this hardcoded id;
  // extend PickObject.action with `string object_id` and pull it from the
  // goal instead.
  pick_object::PickConfig config;
  config.object_id = "cylinder_1";
  // Tie the MTC task name to this action goal's UUID for end-to-end traceability.
  config.task_id = rclcpp_action::to_string(goal_handle->get_goal_id());

  // ---- DIAGNOSTIC: verify the scene contains what we expect ----
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto known = psi.getKnownObjectNames();

    RCLCPP_INFO(node_->get_logger(),
                "PickObject diag: MoveIt scene contains %zu object(s):",
                known.size());
    for (const auto& name : known) {
      RCLCPP_INFO(node_->get_logger(), "  - '%s'", name.c_str());
    }

    if (std::find(known.begin(), known.end(), config.object_id) == known.end()) {
      result->success = false;
      result->message = "OBJECT_NOT_IN_SCENE: '" + config.object_id +
                        "' not present in MoveIt planning scene";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // Dump what MoveIt actually has for this object.
    // CollisionObject normalization: MoveIt typically hoists the position into
    // co.pose and leaves co.primitive_poses[0] as identity. So we print both.
    const auto objects = psi.getObjects({config.object_id});
    if (auto it = objects.find(config.object_id); it != objects.end()) {
      const auto& co = it->second;
      const auto& obj = co.pose.position;
      RCLCPP_INFO(node_->get_logger(),
                  "PickObject diag: '%s' frame='%s' object_pose=(%.3f, %.3f, %.3f)",
                  co.id.c_str(), co.header.frame_id.c_str(),
                  obj.x, obj.y, obj.z);
      if (!co.primitive_poses.empty()) {
        const auto& pp = co.primitive_poses[0].position;
        RCLCPP_INFO(node_->get_logger(),
                    "PickObject diag:   primitive[0]_pose (relative)=(%.3f, %.3f, %.3f)",
                    pp.x, pp.y, pp.z);
      }
    }
  }

  // Grasp frame transform: defines how the IK target sits relative to the TCP.
  //
  // Derivation from RViz observation:
  //   - fer_hand_tcp's local Z points DOWN in world (hand fingers face down).
  //   - GenerateGraspPose generates targets with Z pointing UP in world.
  //   - IK constraint: IK frame in world == target pose in world.
  //   - Therefore IK frame Z must be opposite to TCP Z → rotate 180° around X.
  //
  // No translation: the TCP is already at the fingertip — exactly the point
  // we want to coincide with the object's grasp position.
  {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
    config.grasp_frame_transform = t;
  }

  // Lift direction: world +Z, scaled by approach_height.
  config.lift_direction.header.frame_id = "base";
  config.lift_direction.vector.z = 1.0;
  config.lift_min_distance = std::max(0.05f, goal->approach_height * 0.5f);
  config.lift_max_distance = std::max(0.10f, goal->approach_height);

  // ---- Plan ----
  feedback->current_phase = "planning";
  feedback->progress = 0.1f;
  goal_handle->publish_feedback(feedback);

  const auto plan_result = picker_->plan_pick(config);
  if (plan_result.status != pick_object::PlanResult::Status::Success) {
    result->success = false;
    result->message = "PLANNING_FAILED: " + plan_result.failure_reason;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "PickObject: planning failed");
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

  const auto exec_result = picker_->execute_pick();

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "CANCELLED";
    goal_handle->canceled(result);
    return;
  }

  if (exec_result.status != pick_object::ExecuteResult::Status::Success) {
    result->success = false;
    result->message = "EXECUTION_FAILED: " + exec_result.failure_reason;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "PickObject: execution failed");
    return;
  }

  feedback->current_phase = "done";
  feedback->progress = 1.0f;
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "OK";
  goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "PickObject: succeeded");
}

}  // namespace fer_skills

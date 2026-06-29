#pragma once

// Shared types and utilities for the MTC-based skills (pick / place).
//
// Design note — pick and place are deliberately kept as SEPARATE tasks/classes
// so that a Behavior Tree can re-trigger a pick on gripper failure and retry
// from the arm's current pose, without dragging a whole pick+place plan along.
// The price of that split is that the place task can no longer point its
// GeneratePlacePose at the pick task's `attach_object` stage. Instead it must
// monitor its OWN CurrentState, which snapshots the live PlanningScene — and
// after a successful pick that scene already holds the object attached to the
// hand. This header centralises the bits both classes share.

#include <cstddef>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <rclcpp/logger.hpp>

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/task_constructor/stages/connect.h>


namespace mtc = moveit::task_constructor;

namespace mtc_common {

// ---------------------------------------------------------------------------
// Result / status messages returned to the caller (skill server, BT).
// ---------------------------------------------------------------------------

struct PlanResult {
    enum class Status { Success, Failed, InitError };
    Status status = Status::Failed;
    std::string failed_stage;
    std::string failure_reason;
    std::size_t solutions_found = 0;
};

struct ExecuteResult {
    enum class Status { Success, NoPlan, ExecutionFailed };
    Status status = Status::ExecutionFailed;
    std::string failure_reason;
    int moveit_error_code = 0;
};

// ---------------------------------------------------------------------------
// Configs — the seam through which the Behavior Tree injects all high-level
// decisions (which object, what the grasp/place pose is, approach/retreat
// directions, tolerances). The MTC classes themselves hardcode no poses;
// everything geometric arrives here.
// ---------------------------------------------------------------------------

struct PickConfig {
    std::string object_id;

    // How the IK target frame sits relative to the TCP for grasping.
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();

    // GenerateGraspPose sampling + hand named targets.
    double angle_delta_rad = M_PI / 12;
    std::string pre_grasp_pose = "open";
    std::string grasp_pose_name = "close";

    // IK sampling.
    int max_ik_solutions = 4;
    double min_ik_solution_distance = 1.0;

    // Connect (move-to-pick) planning timeout.
    double connect_timeout_s = 5.0;

    // Lift after grasping.
    double lift_min_distance = 0.10;
    double lift_max_distance = 0.30;
    geometry_msgs::msg::Vector3Stamped lift_direction;

    // Optional traceability id (e.g. the action goal UUID). Folded into the MTC
    // task name so a given plan is traceable back to the request that issued it.
    std::string task_id;
};

struct PlaceConfig {
    std::string object_id;

    // Where the object's reference frame should end up. Supplied by the BT
    // (frame_id + pose). The place IK frame is the object itself, so the
    // object lands exactly at this pose.
    geometry_msgs::msg::PoseStamped place_pose;

    // Hand named target used to release the object.
    std::string release_pose = "open";

    // IK sampling for the place pose.
    int max_ik_solutions = 4;
    double min_ik_solution_distance = 1.0;

    // Connect (move-to-place) planning timeout.
    double connect_timeout_s = 5.0;

    // Retreat after releasing the object.
    geometry_msgs::msg::Vector3Stamped retreat_direction;
    double retreat_min_distance = 0.05;
    double retreat_max_distance = 0.15;

    // Optional traceability id (e.g. the action goal UUID). Folded into the MTC
    // task name so a given plan is traceable back to the request that issued it.
    std::string task_id;
};

// ---------------------------------------------------------------------------
// Utilities shared by PickObject / PlaceObject.
// ---------------------------------------------------------------------------

// Build a traceable task name, e.g. "pick_object [cylinder_1] #<uuid>".
// object_id and task_id are appended only when non-empty.
std::string make_task_name(
    const std::string& base,
    const std::string& object_id,
    const std::string& task_id);

// Set the task name and the standard {group, eef, ik_frame} properties.
// Does NOT (re)load the robot model — call task.loadRobotModel() once in the
// constructor; the model survives task.clear().
void apply_default_properties(
    mtc::Task& task,
    const std::string& task_name,
    const std::string& arm_group,
    const std::string& hand_group,
    const std::string& tcp_frame);

// Link names (with collision geometry) of the given hand group — the set that
// must be allowed/forbidden to collide with the manipulated object.
std::vector<std::string> hand_collision_links(
    const mtc::Task& task,
    const std::string& hand_group);

// init() + plan() the task and translate the outcome into a PlanResult.
// `label` is used purely for log lines (e.g. "Pick", "Place").
PlanResult plan_task(
    mtc::Task& task,
    const rclcpp::Logger& logger,
    int max_solutions,
    const std::string& label);

// Execute the best (front) solution and translate the outcome into an
// ExecuteResult. Returns NoPlan if the task has no solutions.
ExecuteResult execute_task(
    mtc::Task& task,
    const rclcpp::Logger& logger,
    const std::string& label);

// ---------------------------------------------------------------------------
// Stage factories — dumb builders for individual stages shared by pick/place.
// They take plain primitives (no Pick/PlaceConfig) so they stay agnostic of
// either skill; the skill classes own the orchestration (ordering, monitored
// pointers, containers).
// ---------------------------------------------------------------------------

// Move a joint group to a named target pose (e.g. open / close the hand).
std::unique_ptr<mtc::Stage> make_move_to_named(
    const std::string& name,
    const std::string& group,
    const std::string& goal,
    const mtc::solvers::PlannerInterfacePtr& planner);

// Connect: free-motion bridge into the next interface state.
std::unique_ptr<mtc::Stage> make_connect(
    const std::string& name,
    const mtc::stages::Connect::GroupPlannerVector& groups,
    double timeout_s);

// Relative cartesian motion along a direction (e.g. lift / retreat).
std::unique_ptr<mtc::Stage> make_relative(
    const std::string& name,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& ik_frame,
    const geometry_msgs::msg::Vector3Stamped& direction,
    double min_distance,
    double max_distance,
    const std::string& marker_ns);

// Allow (true) or forbid (false) collisions between an object and the given
// links in the ACM.
std::unique_ptr<mtc::Stage> make_modify_collisions(
    const std::string& name,
    const std::string& object_id,
    const std::vector<std::string>& links,
    bool allow);

// Attach (true) or detach (false) an object to/from the hand group.
std::unique_ptr<mtc::Stage> make_attach(
    const std::string& name,
    const std::string& object_id,
    const std::string& hand_group,
    bool attach);

}  // namespace mtc_common

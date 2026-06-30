#include "fer_skills/mtc_place_object.hpp"

#include <utility>

#include <rclcpp/logging.hpp>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>


namespace place_object
{

PlaceObject::PlaceObject(
    rclcpp::Node::SharedPtr node,
    std::string arm_group,
    std::string hand_group,
    std::string tcp_frame,
    moveit::core::RobotModelConstPtr robot_model,
    std::shared_ptr<const fer_skills::MTCPlanners> planners
):
    node_{std::move(node)},
    arm_group_{std::move(arm_group)},
    hand_group_{std::move(hand_group)},
    tcp_frame_{std::move(tcp_frame)},
    planners_{std::move(planners)}
{
    RCLCPP_INFO(node_->get_logger(),
        "Place Object Planner initialized with arm group: %s, "
        "hand group: %s and tcp planning frame: %s",
        arm_group_.c_str(), hand_group_.c_str(), tcp_frame_.c_str());

    // Set the model once; it survives task_.clear() so we never reseed.
    task_.setRobotModel(std::move(robot_model));
    mtc_common::apply_default_properties(
        task_, "Place Object", arm_group_, hand_group_, tcp_frame_);
}

void PlaceObject::clear_task()
{
    task_.clear();
    mtc_common::apply_default_properties(
        task_, "Place Object", arm_group_, hand_group_, tcp_frame_);
}

PlanResult PlaceObject::plan_place(const PlaceConfig& config)
{
    clear_task();
    task_.setName(
        mtc_common::make_task_name("place_object", config.object_id, config.task_id));

    // CurrentState snapshots the LIVE planning scene. Because pick and place
    // are separate tasks, we cannot point GeneratePlacePose at the pick task's
    // attach_object stage. Instead we monitor this CurrentState: after a
    // successful pick the global PlanningScene already holds the object
    // attached to the hand, so this stage's scene carries that attachment —
    // which GeneratePlacePose needs to locate the object and compute the place
    // IK. We keep a NON-OWNING pointer to it and hand ownership to the task, so
    // the pointer stays valid for the whole task lifetime (no dangling).
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    mtc::Stage* current_state_ptr = stage_state_current.get();
    task_.add(std::move(stage_state_current));

    // Bridge from wherever the arm currently is to the place pose.
    task_.add(mtc_common::make_connect(
        "move to place", {{arm_group_, planners_->sampling}}, config.connect_timeout_s));

    // Everything from the place pose onward lives in a serial container so the
    // {eef, group, ik_frame} properties propagate to its children.
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task_.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    place->insert(make_place_pose(config, current_state_ptr));
    place->insert(mtc_common::make_move_to_named(
        "open hand", hand_group_, config.release_pose, planners_->interpolation));
    // KEEP hand<->object collision allowed through detach + retreat. At the place
    // pose the palm (fer_hand) still geometrically overlaps the object (same
    // overlap pick had to allow to grasp). Detaching removes the attachment's
    // touch-link exemption, so without an explicit allow the residual overlap
    // would be flagged as a hard collision. The hand only clears the object once
    // it has retreated.
    place->insert(mtc_common::make_modify_collisions(
        "allow collision (hand,object)", config.object_id,
        mtc_common::hand_collision_links(task_, hand_group_), /*allow=*/true));
    place->insert(mtc_common::make_attach(
        "detach object", config.object_id, hand_group_, /*attach=*/false));
    place->insert(mtc_common::make_relative(
        "retreat", planners_->cartesian, tcp_frame_, config.retreat_direction,
        config.retreat_min_distance, config.retreat_max_distance, "retreat"));
    // Now that the hand has retreated clear of the object, restore normal
    // collision checking between them so future motions treat it as an obstacle.
    place->insert(mtc_common::make_modify_collisions(
        "forbid collision (hand,object)", config.object_id,
        mtc_common::hand_collision_links(task_, hand_group_), /*allow=*/false));

    task_.add(std::move(place));

    // Traceability: ties this plan to the originating action goal (the name
    // embeds object id + goal UUID). Visible in logs and failure dumps.
    RCLCPP_INFO(node_->get_logger(),
                "Planning MTC task named: '%s'", task_.name().c_str());

    return mtc_common::plan_task(task_, node_->get_logger(), 5, "Place");
}

ExecuteResult PlaceObject::execute_place()
{
    return mtc_common::execute_task(task_, node_->get_logger(), "Place");
}

std::unique_ptr<mtc::Stage> PlaceObject::make_place_pose(
    const PlaceConfig& config, mtc::Stage* current_state_ptr)
{
    // Sample the placement pose for the object.
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject(config.object_id);
    stage->setPose(config.place_pose);
    // Monitor the scene that holds the attached object (see plan_place).
    stage->setMonitoredStage(current_state_ptr);

    // Compute IK with the OBJECT frame as the IK frame, so the object lands
    // exactly at config.place_pose.
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(config.max_ik_solutions);
    wrapper->setMinSolutionDistance(config.min_ik_solution_distance);
    wrapper->setIKFrame(config.object_id);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    return wrapper;
}

}  // namespace place_object

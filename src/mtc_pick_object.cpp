#include "fer_skills/mtc_pick_object.hpp"

#include <utility>

#include <rclcpp/logging.hpp>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>


namespace pick_object
{
    PickObject::PickObject(
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
            "Pick Object Planner initialized with arm group: %s, "
            "hand group: %s and tcp planning frame: %s",
            arm_group_.c_str(), hand_group_.c_str(), tcp_frame_.c_str());

        // Set the model once; it survives task_.clear() so we never reseed.
        task_.setRobotModel(std::move(robot_model));
        mtc_common::apply_default_properties(
            task_, "Pick Object", arm_group_, hand_group_, tcp_frame_);
    }

    void PickObject::clear_task()
    {
        task_.clear();
        mtc_common::apply_default_properties(
            task_, "Pick Object", arm_group_, hand_group_, tcp_frame_);
    }

    PlanResult PickObject::plan_pick(const PickConfig& config)
    {
        clear_task();
        task_.setName(
            mtc_common::make_task_name("pick_object", config.object_id, config.task_id));

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task_.add(std::move(stage_state_current));
        task_.add(mtc_common::make_move_to_named(
            "open hand", hand_group_, config.pre_grasp_pose, planners_->interpolation));

        // allow_collision (hand, object) at TASK level — see prior note about
        // backward-pull propagation. We also keep a non-owning pointer to it
        // because the grasp-pose generator's collision check uses the scene
        // of its MONITORED stage. We must monitor a stage whose scene contains
        // the ACM allowance; CurrentState's scene predates the modification.
        auto allow_collision_stage = mtc_common::make_modify_collisions(
            "allow collision (hand, object)", config.object_id,
            mtc_common::hand_collision_links(task_, hand_group_), /*allow=*/true);
        mtc::Stage* allow_collision_ptr = allow_collision_stage.get();
        task_.add(std::move(allow_collision_stage));

        task_.add(mtc_common::make_connect(
            "move_to_pick", {{arm_group_, planners_->sampling}}, config.connect_timeout_s));

        auto grasp = std::make_unique<mtc::SerialContainer>("pick_object");
        task_.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Monitor allow_collision_ptr (not current_state_ptr) so the IK's
        // collision check evaluates against the scene with the ACM allowance.
        grasp->insert(make_grasp_pose(config, allow_collision_ptr));
        grasp->insert(mtc_common::make_move_to_named(
            "close hand", hand_group_, config.grasp_pose_name, planners_->interpolation));
        grasp->insert(mtc_common::make_attach(
            "attach object", config.object_id, hand_group_, /*attach=*/true));
        grasp->insert(mtc_common::make_relative(
            "lift object", planners_->cartesian, hand_group_, config.lift_direction,
            config.lift_min_distance, config.lift_max_distance, "lift_object"));

        task_.add(std::move(grasp));

        // Traceability: ties this plan to the originating action goal (the name
        // embeds object id + goal UUID). Visible in logs and failure dumps.
        RCLCPP_INFO(node_->get_logger(),
                    "Planning MTC task named: '%s'", task_.name().c_str());

        return mtc_common::plan_task(task_, node_->get_logger(), 5, "Pick");
    }

    ExecuteResult PickObject::execute_pick()
    {
        return mtc_common::execute_task(task_, node_->get_logger(), "Pick");
    }

    std::unique_ptr<mtc::Stage> PickObject::make_grasp_pose(
        const PickConfig& config, mtc::Stage* current_state_ptr)
    {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp_pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose(config.pre_grasp_pose);
        stage->setObject(config.object_id);
        stage->setAngleDelta(config.angle_delta_rad);
        stage->setMonitoredStage(current_state_ptr);  // Hook into current state

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(config.max_ik_solutions);
        wrapper->setMinSolutionDistance(config.min_ik_solution_distance);
        wrapper->setIKFrame(config.grasp_frame_transform, tcp_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

        return wrapper;
    }

}  // namespace pick_object

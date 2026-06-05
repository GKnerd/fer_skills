#include "fer_skills/mtc_pick_object.hpp"

#include <sstream>
#include <utility>

#include <rclcpp/logging.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>



namespace pick_object 
{
    PickObject::PickObject(
        rclcpp::Node::SharedPtr node,
        std::string arm_group,
        std::string hand_group,
        std::string tcp_frame,
        std::shared_ptr<const fer_skills::MTCPlanners> planners
    ):
    node_{std::move(node)},
    arm_group_{arm_group},
    hand_group_{hand_group},
    tcp_frame_{tcp_frame},
    planners_{std::move(planners)}
    {
        RCLCPP_INFO(node_->get_logger(),
        "Pick Object Planner initialized with arm group: %.*s, \
        hand_group: %.*s and tcp planning frame: %.*s", 
        static_cast<int>(arm_group.size()), arm_group.data(), 
        static_cast<int>(hand_group.size()), hand_group.data(), 
        static_cast<int>(tcp_frame.size()), tcp_frame.data()
        );

        //TODO: Add UUID to the task maybe?
        task_.stages()->setName("Pick Object");
        task_.loadRobotModel(node_);
        task_.setProperty("group", arm_group);
        task_.setProperty("eef", hand_group);
        task_.setProperty("ik_frame", tcp_frame);
        
    }

    std::unique_ptr<mtc::Stage> PickObject::make_open_hand(const PickConfig& config)
    {
        auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand", planners_->interpolation);
            stage_open_hand->setGroup(hand_group_);
            stage_open_hand->setGoal(config.pre_grasp_pose);
        
            return stage_open_hand;
    };

    std::unique_ptr<mtc::Stage> PickObject::make_close_hand(const PickConfig& config)
    {
        auto stage_close_hand = 
            std::make_unique<mtc::stages::MoveTo>("close hand", planners_->interpolation);
        
        stage_close_hand->setGroup(hand_group_);
        stage_close_hand->setGoal(config.grasp_pose_name);
        
        return stage_close_hand;
    };

    std::unique_ptr<mtc::Stage> PickObject::make_connect(const PickConfig& config)
    {
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move_to_pick",
            mtc::stages::Connect::GroupPlannerVector{ {arm_group_, planners_->sampling}}
        );

        stage_move_to_pick->setTimeout(config.connect_timeout_s);
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        // TODO: Add failure mechanism
        //stage_move_to_pick->explainFailure(std::ostream &)
        return stage_move_to_pick;
    };
    
    std::unique_ptr<mtc::Stage> PickObject::make_allow_collision(
        const PickConfig& config, 
        std::vector<std::string> links_w_collision_geometry)
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, object)");
        stage->allowCollisions(config.object_id, links_w_collision_geometry, true);

        return stage;
    };
    
    std::unique_ptr<mtc::Stage> PickObject::make_attach_object(const PickConfig& config)
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        stage->attachObject(config.object_id, hand_group_);
        
        return stage;
    };
    
    std::unique_ptr<mtc::Stage> PickObject::make_lift(const PickConfig& config)
    {
        auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", planners_->cartesian);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.lift_min_distance, config.lift_max_distance);
        stage->setIKFrame(hand_group_);
        stage->properties().set("marker_ns", "lift_object");
        stage->setDirection(config.lift_direction);

        return stage;
    };
    

    std::unique_ptr<mtc::Stage> PickObject::make_grasp_pose(const PickConfig& config, mtc::Stage* current_state_ptr)
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
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        return wrapper;
    };
        
    void PickObject::clear_task()
    {
        task_.clear();
        task_.stages()->setName("Pick Object");
        task_.setProperty("group",    arm_group_);
        task_.setProperty("eef",      hand_group_);
        task_.setProperty("ik_frame", tcp_frame_);
    }

    PlanResult PickObject::plan_pick(const PickConfig& config)
    {
        PlanResult result;
        clear_task();

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task_.add(std::move(stage_state_current));
        task_.add(make_open_hand(config));

        // allow_collision (hand, object) at TASK level — see prior note about
        // backward-pull propagation. We also keep a non-owning pointer to it
        // because the grasp-pose generator's collision check uses the scene
        // of its MONITORED stage. We must monitor a stage whose scene contains
        // the ACM allowance; CurrentState's scene predates the modification.
        auto allow_collision_stage = make_allow_collision(
            config,
            task_.getRobotModel()->getJointModelGroup(hand_group_)->getLinkModelNamesWithCollisionGeometry());
        mtc::Stage* allow_collision_ptr = allow_collision_stage.get();
        task_.add(std::move(allow_collision_stage));

        task_.add(make_connect(config));

        auto grasp = std::make_unique<mtc::SerialContainer>("pick_object");
        task_.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Monitor allow_collision_ptr (not current_state_ptr) so the IK's
        // collision check evaluates against the scene with the ACM allowance.
        grasp->insert(make_grasp_pose(config, allow_collision_ptr));
        grasp->insert(make_close_hand(config));
        grasp->insert(make_attach_object(config));
        grasp->insert(make_lift(config));

        task_.add(std::move(grasp));

        try {
            task_.init();
            const auto plan_code = task_.plan(5);
            const bool plan_ok =
                (plan_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
            result.solutions_found = task_.solutions().size();

            if (plan_ok && result.solutions_found > 0) {
                result.status = PlanResult::Status::Success;
                RCLCPP_INFO(node_->get_logger(),
                            "Pick planning succeeded (%zu solutions)",
                            result.solutions_found);
                return result;
            }

            // Failed: capture a human-readable dump of the task state so the
            // caller / BT has something actionable. The BT can grep for stage
            // names like "grasp pose IK" if it wants to branch on cause.
            result.status = PlanResult::Status::Failed;
            std::ostringstream oss;
            task_.printState(oss);
            result.failure_reason = oss.str();
            RCLCPP_WARN(node_->get_logger(),
                        "Pick planning failed:\n%s",
                        result.failure_reason.c_str());
            return result;

        } catch (const std::exception& e) {
            result.status = PlanResult::Status::InitError;
            result.failure_reason = e.what();
            RCLCPP_ERROR(node_->get_logger(),
                         "Pick planning threw during init: %s", e.what());
            return result;
        }
    }

    ExecuteResult PickObject::execute_pick()
    {
        ExecuteResult result;

        if (task_.solutions().empty()) {
            result.status = ExecuteResult::Status::NoPlan;
            result.failure_reason =
                "execute_pick called without a successful plan_pick";
            RCLCPP_ERROR(node_->get_logger(), "%s",
                         result.failure_reason.c_str());
            return result;
        }

        // Pick the best (front) solution — solutions are ranked by cost.
        const auto& solution = **task_.solutions().begin();

        // Blocks until execution completes or fails.
        const auto error_code = task_.execute(solution);
        result.moveit_error_code = error_code.val;

        if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            result.status = ExecuteResult::Status::Success;
            RCLCPP_INFO(node_->get_logger(), "Pick executed successfully");
            return result;
        }

        result.status = ExecuteResult::Status::ExecutionFailed;
        result.failure_reason =
            "MoveIt execution failed (MoveItErrorCode: "
            + std::to_string(error_code.val) + ")";
        RCLCPP_ERROR(node_->get_logger(), "%s",
                     result.failure_reason.c_str());
        return result;
    }

} // namespace pick_object

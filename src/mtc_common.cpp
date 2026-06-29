#include "fer_skills/mtc_common.hpp"

#include <sstream>
#include <string>

#include <rclcpp/logging.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>

#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>


namespace mtc_common {

std::string make_task_name(
    const std::string& base,
    const std::string& object_id,
    const std::string& task_id)
{
    std::string name = base;
    if (!object_id.empty()) {
        name += " [" + object_id + "]";
    }
    if (!task_id.empty()) {
        name += " #" + task_id;
    }
    return name;
}

void apply_default_properties(
    mtc::Task& task,
    const std::string& task_name,
    const std::string& arm_group,
    const std::string& hand_group,
    const std::string& tcp_frame)
{
    task.stages()->setName(task_name);
    task.setProperty("group", arm_group);
    task.setProperty("eef", hand_group);
    task.setProperty("ik_frame", tcp_frame);
}

std::vector<std::string> hand_collision_links(
    const mtc::Task& task,
    const std::string& hand_group)
{
    return task.getRobotModel()
        ->getJointModelGroup(hand_group)
        ->getLinkModelNamesWithCollisionGeometry();
}

PlanResult plan_task(
    mtc::Task& task,
    const rclcpp::Logger& logger,
    int max_solutions,
    const std::string& label)
{
    PlanResult result;

    try {
        task.init();
        const auto plan_code = task.plan(max_solutions);
        result.solutions_found = task.solutions().size();

        const bool plan_ok =
            (plan_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

        if (plan_ok && result.solutions_found > 0) {
            result.status = PlanResult::Status::Success;
            RCLCPP_INFO(logger, "%s planning succeeded (%zu solutions)",
                        label.c_str(), result.solutions_found);
            return result;
        }

        // Failed: capture a human-readable dump of the task state so the
        // caller / BT has something actionable. The BT can grep for stage
        // names (e.g. "grasp pose IK", "place pose IK") to branch on cause.
        result.status = PlanResult::Status::Failed;
        std::ostringstream oss;
        task.printState(oss);
        result.failure_reason = oss.str();
        RCLCPP_WARN(logger, "%s planning failed:\n%s",
                    label.c_str(), result.failure_reason.c_str());
        return result;

    } catch (const std::exception& e) {
        result.status = PlanResult::Status::InitError;
        result.failure_reason = e.what();
        RCLCPP_ERROR(logger, "%s planning threw during init: %s",
                     label.c_str(), e.what());
        return result;
    }
}

ExecuteResult execute_task(
    mtc::Task& task,
    const rclcpp::Logger& logger,
    const std::string& label)
{
    ExecuteResult result;

    if (task.solutions().empty()) {
        result.status = ExecuteResult::Status::NoPlan;
        result.failure_reason =
            label + " execute called without a successful plan";
        RCLCPP_ERROR(logger, "%s", result.failure_reason.c_str());
        return result;
    }

    // Pick the best (front) solution — solutions are ranked by cost.
    const auto& solution = **task.solutions().begin();

    // Blocks until execution completes or fails.
    const auto error_code = task.execute(solution);
    result.moveit_error_code = error_code.val;

    if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        result.status = ExecuteResult::Status::Success;
        RCLCPP_INFO(logger, "%s executed successfully", label.c_str());
        return result;
    }

    result.status = ExecuteResult::Status::ExecutionFailed;
    result.failure_reason =
        "MoveIt execution failed (MoveItErrorCode: "
        + std::to_string(error_code.val) + ")";
    RCLCPP_ERROR(logger, "%s", result.failure_reason.c_str());
    return result;
}

std::unique_ptr<mtc::Stage> make_move_to_named(
    const std::string& name,
    const std::string& group,
    const std::string& goal,
    const mtc::solvers::PlannerInterfacePtr& planner)
{
    auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
    stage->setGroup(group);
    stage->setGoal(goal);
    return stage;
}

std::unique_ptr<mtc::Stage> make_connect(
    const std::string& name,
    const mtc::stages::Connect::GroupPlannerVector& groups,
    double timeout_s)
{
    auto stage = std::make_unique<mtc::stages::Connect>(name, groups);
    stage->setTimeout(timeout_s);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    return stage;
}

std::unique_ptr<mtc::Stage> make_relative(
    const std::string& name,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const std::string& ik_frame,
    const geometry_msgs::msg::Vector3Stamped& direction,
    double min_distance,
    double max_distance,
    const std::string& marker_ns)
{
    auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(min_distance, max_distance);
    stage->setIKFrame(ik_frame);
    stage->properties().set("marker_ns", marker_ns);
    stage->setDirection(direction);
    return stage;
}

std::unique_ptr<mtc::Stage> make_modify_collisions(
    const std::string& name,
    const std::string& object_id,
    const std::vector<std::string>& links,
    bool allow)
{
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    stage->allowCollisions(object_id, links, allow);
    return stage;
}

std::unique_ptr<mtc::Stage> make_attach(
    const std::string& name,
    const std::string& object_id,
    const std::string& hand_group,
    bool attach)
{
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    if (attach) {
        stage->attachObject(object_id, hand_group);
    } else {
        stage->detachObject(object_id, hand_group);
    }
    return stage;
}

}  // namespace mtc_common

#pragma once

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>

#include "fer_skills/mtc_common.hpp"
#include "fer_skills/mtc_planners.hpp"


namespace mtc = moveit::task_constructor;

namespace pick_object {

// Shared types live in mtc_common (see mtc_common.hpp). Re-export them here so
// existing `pick_object::PickConfig` / `pick_object::PlanResult` references keep
// resolving.
using mtc_common::ExecuteResult;
using mtc_common::PickConfig;
using mtc_common::PlanResult;


class PickObject
{
public:
    PickObject(
        rclcpp::Node::SharedPtr node,
        std::string arm_group,
        std::string hand_group,
        std::string tcp_frame,
        moveit::core::RobotModelConstPtr robot_model,
        std::shared_ptr<const fer_skills::MTCPlanners> planners);

    rclcpp::Node::SharedPtr node() const { return node_; }

    PlanResult plan_pick(const PickConfig& config);
    ExecuteResult execute_pick();
    void clear_task();

private:
    std::unique_ptr<mtc::Stage> make_grasp_pose(
        const PickConfig& config,
        mtc::Stage* current_state_ptr);

    rclcpp::Node::SharedPtr node_;
    std::string arm_group_;
    std::string hand_group_;
    std::string tcp_frame_;
    std::shared_ptr<const fer_skills::MTCPlanners> planners_;
    mtc::Task task_;
};

}  // namespace pick_object

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

namespace place_object {

// Shared types live in mtc_common (see mtc_common.hpp). Re-export them here so
// existing `place_object::PlaceConfig` / `place_object::PlanResult` references
// keep resolving.
using mtc_common::ExecuteResult;
using mtc_common::PlaceConfig;
using mtc_common::PlanResult;


class PlaceObject
{
public:
    PlaceObject(
        rclcpp::Node::SharedPtr node,
        std::string arm_group,
        std::string hand_group,
        std::string tcp_frame,
        moveit::core::RobotModelConstPtr robot_model,
        std::shared_ptr<const fer_skills::MTCPlanners> planners);

    rclcpp::Node::SharedPtr node() const { return node_; }

    PlanResult plan_place(const PlaceConfig& config);
    ExecuteResult execute_place();
    void clear_task();

private:
    std::unique_ptr<mtc::Stage> make_place_pose(
        const PlaceConfig& config,
        mtc::Stage* current_state_ptr);

    rclcpp::Node::SharedPtr node_;
    std::string arm_group_;
    std::string hand_group_;
    std::string tcp_frame_;
    std::shared_ptr<const fer_skills::MTCPlanners> planners_;
    mtc::Task task_;
};

}  // namespace place_object

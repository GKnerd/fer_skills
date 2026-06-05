#pragma once

#include <cstddef>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/node.hpp>

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>

#include "fer_skills/mtc_planners.hpp"


namespace mtc = moveit::task_constructor;

namespace pick_object {

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

struct PickConfig {
    std::string object_id;
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    double angle_delta_rad = M_PI / 12;
    std::string pre_grasp_pose = "open";
    std::string grasp_pose_name = "close";
    int max_ik_solutions = 4;
    double min_ik_solution_distance = 1.0;
    double connect_timeout_s = 5.0;
    double lift_min_distance = 0.10;
    double lift_max_distance = 0.30;
    geometry_msgs::msg::Vector3Stamped lift_direction;
};


class PickObject
{
public:
    PickObject(
        rclcpp::Node::SharedPtr node,
        std::string arm_group,
        std::string hand_group,
        std::string tcp_frame,
        std::shared_ptr<const fer_skills::MTCPlanners> planners);

    rclcpp::Node::SharedPtr node() const { return node_; }

    PlanResult plan_pick(const PickConfig& config);
    ExecuteResult execute_pick();
    void clear_task();

private:
    std::unique_ptr<mtc::Stage> make_open_hand(const PickConfig& config);
    std::unique_ptr<mtc::Stage> make_close_hand(const PickConfig& config);
    std::unique_ptr<mtc::Stage> make_connect(const PickConfig& config);
    std::unique_ptr<mtc::Stage> make_allow_collision(
        const PickConfig& config,
        std::vector<std::string> links_w_collision_geometry);
    std::unique_ptr<mtc::Stage> make_attach_object(const PickConfig& config);
    std::unique_ptr<mtc::Stage> make_lift(const PickConfig& config);
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

} // namespace pick_object

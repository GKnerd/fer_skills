#pragma once

#include <memory>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/node.hpp>
#include <string>
#include "fer_skills/mtc_planners.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>


namespace mtc = moveit::task_constructor;
namespace place_object {


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

struct PlaceConfig {
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


class PlaceObject{
    public:
        PlaceObject(
            rclcpp::Node::SharedPtr node,
            std::string arm_group,
            std::string hand_group,
            std::string tcp_frame,
            std::shared_ptr<const fer_skills::MTCPlanners> planners);
        
        rclcpp::Node::SharedPtr node() const { return node_; }

        PlanResult plan_place(const PlaceConfig& config);
        ExecuteResult execute_place();
        void clear_task();

    private:
        std::unique_ptr<mtc::Stage> make_open_hand(const PlaceConfig& config);
        std::unique_ptr<mtc::Stage> make_close_hand(const PlaceConfig& config);
        std::unique_ptr<mtc::Stage> make_move_to_place(const PlaceConfig& config);
        std::unique_ptr<mtc::Stage> make_allow_collision(
            const PlaceConfig& config,
            std::vector<std::string> links_w_collision_geometry
        );
        std::unique_ptr<mtc::Stage> make_detach_object(const PlaceConfig& config);
        std::unique_ptr<mtc::Stage> make_retreat(const PlaceConfig& config);
        std::unique_ptr<mtc::Stage> make_place_pose(
            const PlaceConfig& config, 
            mtc::Stage* current_state_ptr
        );

        
        rclcpp::Node::SharedPtr node_;
        std::string arm_group_;
        std::string hand_group_;
        std::string tcp_frame_;
        std::shared_ptr<const fer_skills::MTCPlanners> planners_;
        mtc::Task task_;

};

} // namespace place_object

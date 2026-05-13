#pragma once

#include <memory>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "fer_skills/mtc_planners.hpp"

namespace mtc = moveit::task_constructor;

namespace pick_object{

class PickObjectPlanner
{
    public:
        
        PickObjectPlanner(
            rclcpp::Node::SharedPtr node_,
            const std::string_view arm_group,
            const std::string_view hand_group,
            const std::string_view tcp_frame,
            std::shared_ptr<mtc_planners::MTCPlanners> planners_
        );

        rclcpp::Node::SharedPtr node() const { return  node_;}
        
        void setup_planning_scene();
        void plan_pick();
        void execute_pick();

    private:
        mtc::Task create_task;
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;  
        std::shared_ptr<mtc_planners::MTCPlanners> planners_;
};

} // namespace pick_object

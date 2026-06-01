#pragma once

#include <memory>


#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <string>

#include "fer_skills/mtc_planners.hpp"

namespace mtc = moveit::task_constructor;

namespace pick_object{

class PickObject
{
    public:
        
        PickObject(
            rclcpp::Node::SharedPtr node_,
            std::string arm_group,
            std::string hand_group,
            std::string tcp_frame,
            std::shared_ptr<const fer_skills::MTCPlanners> planners_
        );

        rclcpp::Node::SharedPtr node() const {return  node_;}
        
        void setup_planning_scene();
        void plan_pick();
        void execute_pick();

    private:

        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;  
        std::string arm_group_;
        std::string hand_group_;
        std::string tcp_frame_;
        std::shared_ptr<const fer_skills::MTCPlanners> planners_;
};

} // namespace pick_object

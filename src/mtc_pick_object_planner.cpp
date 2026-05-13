#include "fer_skills/mtc_pick_object_planner.hpp"
#include "fer_skills/mtc_planners.hpp"

#include <algorithm>
#include <memory>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>


namespace pick_object 
{
    PickObjectPlanner::PickObjectPlanner(
        rclcpp::Node::SharedPtr node_,
        const std::string_view arm_group,
        const std::string_view hand_group,
        const std::string_view tcp_frame,
        std::shared_ptr<mtc_planners::MTCPlanners> planners_
    ):
    node_{std::move(node_)}
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
        task_.setProperty("arm_group", arm_group);
        task_.setProperty("eef", hand_group);
        task_.setProperty("ik_frame", tcp_frame);
        
    }

    void PickObjectPlanner::setup_planning_scene()
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "world";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = { 0.1, 0.02 };

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = -0.25;
        pose.orientation.w = 1.0;
        object.pose = pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(object);
    }

    void PickObjectPlanner::plan_pick()
    {
        // Init the current state ptr
        mtc::Stage* current_state_ptr = nullptr;

        // Current State
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");

        // The pointer now points to the current state
        current_state_ptr = stage_state_current.get();
        
        task_.add(std::move(stage_state_current));

        // TODO 3: Maybe we need a check here, if the hand is already open?
        auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand",planners_->interpolation);
            stage_open_hand->setGroup(hand_group);
            stage_open_hand->setGoal("open");
            task_.add(std::move(stage_open_hand));
    }   

} // namespace pick_object

#include "fer_skills/mtc_place_object.hpp"

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <memory>

namespace place_object 
{

    PlaceObject::PlaceObject(
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
        task_.stages()->setName("Place Object");
        task_.loadRobotModel(node_);
        task_.setProperty("group", arm_group);
        task_.setProperty("eef", hand_group);
        task_.setProperty("ik_frame", tcp_frame);
        
    }

    PlanResult PlaceObject::plan_place(const PlaceConfig& config)
    {
        PlanResult result;
        clear_task();
        
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task_.add(std::move(stage_state_current));
        task_.add(make_open_hand(config));

        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task_.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame"});
        
        auto allow_collision_stage = make_allow_collision(
            config,
            task_.getRobotModel()->getJointModelGroup(hand_group_)->getLinkModelNamesWithCollisionGeometry());
        mtc::Stage* current_state_ptr = allow_collision_stage.get();

        task_.insert(make_place_pose(config, current_state_ptr));
        task_.insert(make_move_to_place(config));
        task_.insert(make_open_hand(config));
        task_.insert(make_detach_object(config));


        return result;
    }

    std::unique_ptr<mtc::Stage> PlaceObject::make_move_to_place(const PlaceConfig& config)
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{ 
                { arm_group_, planners_->sampling },
                { hand_group_, planners_->interpolation } 
            }
        );
        stage->setTimeout(config.connect_timeout_s);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        
        return stage;
    }

    
    std::unique_ptr<mtc::Stage> PlaceObject::make_open_hand(const PlaceConfig& config)
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand", planners_->interpolation);
        stage->setGroup(hand_group_);
        stage->setGoal("open");
        
        return stage;
    }

    
    std::unique_ptr<mtc::Stage> PlaceObject::make_allow_collision(
        const PlaceConfig& config,
        std::vector<std::string> links_w_collision_geometry)
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        stage->allowCollisions(config.object_id,
            links_w_collision_geometry, 
            false);
        return stage;
    }

  
    std::unique_ptr<mtc::Stage> PlaceObject::make_detach_object(const PlaceConfig& config)
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
        stage->detachObject(config.object_id, hand_group_);
        return stage;
    }


    std::unique_ptr<mtc::Stage> PlaceObject::make_retreat(const PlaceConfig& config)
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", planners_->cartesian);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(config.lift_min_distance, config.lift_max_distance);
        stage->setIKFrame(hand_group_);
        stage->properties().set("marker_ns", "retreat");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.x = -0.5;
        stage->setDirection(vec);

        return stage;
    }   

    std::unique_ptr<mtc::Stage> PlaceObject::make_place_pose(const PlaceConfig& config, mtc::Stage* current_state_ptr)
    {
        // Sample place pose
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(config.object_id);

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "object";
        target_pose_msg.pose.position.y = 0.5;
        target_pose_msg.pose.orientation.w = 1.0;
        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(current_state_ptr);  // Hook into attach_object_stage

        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(config.max_ik_solutions);
        wrapper->setMinSolutionDistance(config.min_ik_solution_distance);
        wrapper->setIKFrame(config.object_id);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        
        return wrapper;
    }


} //namespace place_object

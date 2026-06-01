#include "fer_skills/mtc_pick_object.hpp"
#include "fer_skills/mtc_planners.hpp"


#include <memory>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_random_pose.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <utility>


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

    void PickObject::setup_planning_scene()
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

    void PickObject::plan_pick()
    {
        mtc::Stage* current_state_ptr = nullptr;

        // Current State
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");

        // The pointer now points to the current state
        current_state_ptr = stage_state_current.get();
        
        task_.add(std::move(stage_state_current));

        // TODO 3: Maybe we need a check here, if the hand is already open?
        auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand", planners_->interpolation);
            stage_open_hand->setGroup(hand_group_);
            stage_open_hand->setGoal("open");
            task_.add(std::move(stage_open_hand));
    
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move_to_pick",
            mtc::stages::Connect::GroupPlannerVector{ {arm_group_, planners_->sampling}}
        );
        // TO-DO: Fix the hardcoded value
        stage_move_to_pick->setTimeout(5.0);
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task_.add(std::move(stage_move_to_pick));

        mtc::Stage* attach_object_stage = nullptr;

        // Serial Container
        {
            // Grasp Stage
            auto grasp = std::make_unique<mtc::SerialContainer>("pick_object");
            task_.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

            // Sample Grasp Pose
            {
                // Create the Stage for grasping
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp_pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose("open");
                stage->setObject("object");
                stage->setAngleDelta(M_PI / 12);
                stage->setMonitoredStage(current_state_ptr);  // Hook into current state
                
                // Compute the frame transform
                // TODO: I think I can get this externally, so I will need to have to check on this.
                Eigen::Isometry3d grasp_frame_transform;
                Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
                grasp_frame_transform.linear() = q.matrix();
                grasp_frame_transform.translation().z() = 0.1;

                // Compute IK 
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
                // TODO: Hardcoded value needs fixing
                wrapper->setMaxIKSolutions(4);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(grasp_frame_transform, hand_group_);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
                grasp->insert(std::move(wrapper));
            }

            // Allow collision between objects
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                stage->allowCollisions("object",
                                    task_.getRobotModel()->getJointModelGroup(hand_group_)->getLinkModelNamesWithCollisionGeometry(),
                                    true);
                grasp->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", planners_->interpolation);
                stage->setGroup(hand_group_);
                stage->setGoal("close");
                grasp->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
                stage->attachObject("object", hand_group_);
                attach_object_stage = stage.get();
                grasp->insert(std::move(stage));
            }
            {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", planners_->cartesian);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_group_);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            // To-Do: Also needs to be improved
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
            }
            
            task_.add(std::move(grasp));
        }
        
    }   

} // namespace pick_object

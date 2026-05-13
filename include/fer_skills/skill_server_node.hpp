#pragma once

#include <memory>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "fer_skills/action/go_home.hpp"
#include "fer_skills/action/move_to_pose.hpp"
#include "fer_skills/action/pick_object.hpp"
#include "fer_skills/action/place_object.hpp"
#include "fer_skills/action/control_gripper.hpp"


namespace fer_skills
{

//TO-DO: make the hand move group interface pointer argument optional, if possible 
class SkillServerNode
{
public:

  using GoHome          = fer_skills::action::GoHome;
  using MoveToPose      = fer_skills::action::MoveToPose;
  using PickObject      = fer_skills::action::PickObject;
  using PlaceObject     = fer_skills::action::PlaceObject;
  using ControlGripper  = fer_skills::action::ControlGripper;

  using GoalHandleGoHome          = rclcpp_action::ServerGoalHandle<GoHome>;
  using GoalHandleMoveToPose      = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using GoalHandlePickObject      = rclcpp_action::ServerGoalHandle<PickObject>;
  using GoalHandlePlaceObject     = rclcpp_action::ServerGoalHandle<PlaceObject>;
  using GoalHandleControlGripper  = rclcpp_action::ServerGoalHandle<ControlGripper>;

  SkillServerNode(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand);

  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  // GoHome Handles
  rclcpp_action::GoalResponse handle_go_home_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoHome::Goal> goal
  );
  rclcpp_action::CancelResponse handle_go_home_cancel(
    const std::shared_ptr<GoalHandleGoHome> goal_handle
  );
  void handle_go_home_accepted(const std::shared_ptr<GoalHandleGoHome> goal_handle);
  void execute_go_home(const std::shared_ptr<GoalHandleGoHome> goal_handle);

  // MoveToPose Handles
  rclcpp_action::GoalResponse handle_move_to_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal
  );
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle
  );
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void execute_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  // PickObject Handles
  rclcpp_action::GoalResponse handle_pick_object_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickObject::Goal> goal
  );
  rclcpp_action::CancelResponse handle_pick_object_cancel(
    const std::shared_ptr<GoalHandlePickObject> goal_handle
  );
  void handle_pick_object_accepted(const std::shared_ptr<GoalHandlePickObject> goal_handle);
  void execute_pick_object(const std::shared_ptr<GoalHandlePickObject> goal_handle);

  // PlaceObject Handles
  rclcpp_action::GoalResponse handle_place_object_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const PlaceObject::Goal> goal
  );
  rclcpp_action::CancelResponse handle_place_object_cancel(
    const std::shared_ptr<GoalHandlePlaceObject> goal_handle
  );
  void handle_place_object_accepted(const std::shared_ptr<GoalHandlePlaceObject> goal_handle);
  void execute_place_object(const std::shared_ptr<GoalHandlePlaceObject> goal_handle);

  // ControlGripper Handles
  rclcpp_action::GoalResponse handle_control_gripper_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ControlGripper::Goal> goal
  );
  rclcpp_action::CancelResponse handle_control_gripper_cancel(
    const std::shared_ptr<GoalHandleControlGripper> goal_handle
  );
  void handle_control_gripper_accepted(const std::shared_ptr<GoalHandleControlGripper> goal_handle);
  void execute_control_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle);
  
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_;

  // Action Servers
  rclcpp_action::Server<GoHome>::SharedPtr go_home_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<PickObject>::SharedPtr pick_object_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_object_server_;
  rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_server_;
};

}  // namespace fer_skills

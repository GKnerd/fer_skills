#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "fer_skills/action/go_home.hpp"
#include "fer_skills/action/move_to_pose.hpp"
#include "fer_skills/action/pick_object.hpp"

namespace fer_skills
{

class SkillServerNode
{
public:
  using GoHome = fer_skills::action::GoHome;
  using MoveToPose = fer_skills::action::MoveToPose;
  using PickObject = fer_skills::action::PickObject;

  using GoalHandleGoHome = rclcpp_action::ServerGoalHandle<GoHome>;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using GoalHandlePickObject = rclcpp_action::ServerGoalHandle<PickObject>;

  SkillServerNode(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm);

  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  rclcpp_action::GoalResponse handle_go_home_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoHome::Goal> goal);
  rclcpp_action::CancelResponse handle_go_home_cancel(
    const std::shared_ptr<GoalHandleGoHome> goal_handle);
  void handle_go_home_accepted(const std::shared_ptr<GoalHandleGoHome> goal_handle);
  void execute_go_home(const std::shared_ptr<GoalHandleGoHome> goal_handle);

  rclcpp_action::GoalResponse handle_move_to_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void execute_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  rclcpp_action::GoalResponse handle_pick_object_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickObject::Goal> goal);
  rclcpp_action::CancelResponse handle_pick_object_cancel(
    const std::shared_ptr<GoalHandlePickObject> goal_handle);
  void handle_pick_object_accepted(const std::shared_ptr<GoalHandlePickObject> goal_handle);
  void execute_pick_object(const std::shared_ptr<GoalHandlePickObject> goal_handle);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;

  rclcpp_action::Server<GoHome>::SharedPtr go_home_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<PickObject>::SharedPtr pick_object_server_;
};

}  // namespace fer_skills

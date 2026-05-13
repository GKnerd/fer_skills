#include "fer_skills/skill_server_node.hpp"

#include <functional>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp_action/create_server.hpp>
#include <thread>
#include <utility>


namespace fer_skills
{

SkillServerNode::SkillServerNode(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand
)
: node_(std::move(node)),
  arm_(std::move(arm)),
  hand_(std::move(hand))
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  go_home_server_ = rclcpp_action::create_server<GoHome>(
    node_,
    "go_home",
    std::bind(&SkillServerNode::handle_go_home_goal, this, _1, _2),
    std::bind(&SkillServerNode::handle_go_home_cancel, this, _1),
    std::bind(&SkillServerNode::handle_go_home_accepted, this, _1));

  move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
    node_,
    "move_to_pose",
    std::bind(&SkillServerNode::handle_move_to_pose_goal, this, _1, _2),
    std::bind(&SkillServerNode::handle_move_to_pose_cancel, this, _1),
    std::bind(&SkillServerNode::handle_move_to_pose_accepted, this, _1)
  );

  pick_object_server_ = rclcpp_action::create_server<PickObject>(
    node_,
    "pick_object",
    std::bind(&SkillServerNode::handle_pick_object_goal, this, _1, _2),
    std::bind(&SkillServerNode::handle_pick_object_cancel, this, _1),
    std::bind(&SkillServerNode::handle_pick_object_accepted, this, _1)
  );
  
  place_object_server_ = rclcpp_action::create_server<PlaceObject>(
    node_,
    "place_object",
    std::bind(&SkillServerNode::handle_place_object_goal, this, _1, _2),
    std::bind(&SkillServerNode::handle_place_object_cancel, this, _1),
    std::bind(&SkillServerNode::handle_place_object_accepted, this, _1)
  );

  control_gripper_server_ = rclcpp_action::create_server<ControlGripper>(
    node_,
    "control_gripper",
    std::bind(&SkillServerNode::handle_control_gripper_goal, this, _1, _2),
    std::bind(&SkillServerNode::handle_control_gripper_cancel, this, _1),
    std::bind(&SkillServerNode::handle_control_gripper_accepted, this, _1)
  );
  
  RCLCPP_INFO(
    node_->get_logger(),
    "fer_skills server ready. Actions: /go_home, /move_to_pose, /pick_object, /place_object, /control_gripper");
}

}  // namespace fer_skills

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("fer_skill_server", node_options);

  const std::string arm_group =
    node->has_parameter("arm_group") ?
    node->get_parameter("arm_group").as_string() :
    node->declare_parameter<std::string>("arm_group", "fer_arm");
  
  const std::string hand_group = 
    node->has_parameter("hand_group") ?
    node->get_parameter("hand_group").as_string() :
    node->declare_parameter<std::string>("hand_group","fer_hand");


  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread spinner([executor]() { executor->spin(); });

  // MoveGroupInterface needs a spinning node for service/topic discovery.
  auto arm  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, arm_group);
  auto hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, hand_group);

  RCLCPP_INFO(
    node->get_logger(),
    "Connected to MoveIt group '%s' (planning frame: %s, end effector: %s)",
    arm_group.c_str(),
    arm->getPlanningFrame().c_str(),
    arm->getEndEffectorLink().c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "Connected to MoveIt group '%s' (planning frame: '%s)",
    hand_group.c_str(),
    hand->getPlanningFrame().c_str()
  );

  fer_skills::SkillServerNode server(node, arm, hand);

  spinner.join();
  rclcpp::shutdown();
  return 0;
}

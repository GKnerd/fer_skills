#pragma once 

#include <rclcpp/publisher.hpp>
#include <string>
#include <unordered_map>
#include "Eigen/Eigen"

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <vector>
#include "moveit_msgs/msg/planning_scene.hpp"



namespace fer_skills {

// Planning Scene Data Model
struct PlanningSceneObj
{
    // contains the data of the collision object
    moveit_msgs::msg::CollisionObject object; 
   
    // metadata
    double mass_kg = 0.0;
    bool graspable = false;
    enum class Kind { Target, Obstacle, Fixture } kind = Kind::Obstacle;
    std::optional<Eigen::Isometry3d> preferred_grasp;
};


class PlanningSceneManager
{
    private:
        rclcpp::Node::SharedPtr node_;
        moveit::planning_interface::PlanningSceneInterface psi_;
        rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_monitor_sub_;
        
        // Planning Scene Obj Storage
        std::unordered_map<std::string, PlanningSceneObj> objects_;   
        

    public:
        PlanningSceneManager(
            rclcpp::Node::SharedPtr node,
            moveit::planning_interface::PlanningSceneInterface psi,
            std::unordered_map<std::string, PlanningSceneObj> objects);
        void planning_scene_monitor_cb(moveit_msgs::msg::PlanningScene::SharedPtr msg);

        void add_object(const PlanningSceneObj& obj);
        void rm_scene_object(const PlanningSceneObj& obj);
        
        std::optional<PlanningSceneObj> get_object(const std::string& id) const;
        std::vector<PlanningSceneObj> list_objects(std::optional<PlanningSceneObj::Kind> filter) const;
    

};


} // namespace fer_skills

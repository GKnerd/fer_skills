#include <array>
#include <optional>
#include <unordered_map>
#include <utility>

#include <rclcpp/node.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "fer_skills/planning_scene.hpp"

namespace fer_skills {

    PlanningSceneManager::PlanningSceneManager(rclcpp::Node::SharedPtr node, 
        moveit::planning_interface::PlanningSceneInterface psi,
        std::unordered_map<std::string, PlanningSceneObj> objects):
        node_{std::move(node)},
        psi_{psi},
        objects_{objects}
    {
    };
    
    void PlanningSceneManager::add_object(const PlanningSceneObj& obj)
    {
        objects_[obj.object.id] = obj;
        psi_.applyCollisionObject(obj.object);
    };


    void PlanningSceneManager::rm_scene_object(const PlanningSceneObj& obj)
    {
        objects_.erase(obj.object.id);
        psi_.removeCollisionObjects({obj.object.id});
    };
    

    std::optional<PlanningSceneObj> PlanningSceneManager::get_object(const std::string& id) const
    {   
        auto it = objects_.find(id);
        if (it == objects_.end()) 
        {
            return std::nullopt;
        }
        else 
        {
            return it->second;
        } 
    };
        

    std::vector<PlanningSceneObj> PlanningSceneManager::list_objects(std::optional<PlanningSceneObj::Kind> filter = std::nullopt) const
    {
        std::vector<PlanningSceneObj> out;
        out.reserve(objects_.size());

        for (const auto& [id, obj] : objects_)
        {       
            if (!filter || obj.kind == *filter)
            {
                out.push_back(obj);
            }
        }
        return out;
    };
    

} // namespace fer_skills

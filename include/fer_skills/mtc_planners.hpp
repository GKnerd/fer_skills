# pragma once
#include <memory>

#include "moveit/task_constructor/solvers/cartesian_path.h"
#include "moveit/task_constructor/solvers/joint_interpolation.h"
#include "moveit/task_constructor/solvers/pipeline_planner.h"


namespace mtc = moveit::task_constructor;


namespace fer_skills {

    struct MTCPlanners{
        std::shared_ptr<mtc::solvers::PipelinePlanner> sampling;
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation;
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian;
    };
    
} // namespace fer_skills

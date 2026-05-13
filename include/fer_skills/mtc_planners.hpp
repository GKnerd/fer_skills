# pragma once
#include <memory>

#include "moveit/task_constructor/solvers/cartesian_path.h"
#include "moveit/task_constructor/solvers/joint_interpolation.h"
#include "moveit/task_constructor/solvers/pipeline_planner.h"


namespace mtc = moveit::task_constructor;


namespace mtc_planners {

    struct MTCPlanners{
        std::shared_ptr<mtc::solvers::PipelinePlanner> sampling;
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation;
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian;
    };
    
} // namespace mtc planners 

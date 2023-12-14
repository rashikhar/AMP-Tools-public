#include "DecentralizedMultiAgentRRT.h"
#include "MultiAgentRRT.h"
#include "AMPCore.h"
#include"hw/HW2.h"
#include"hw/HW5.h"
#include"hw/HW8.h"
#include"Eigen/Geometry"
#include"Q1.h"
#include <iostream>
#include <vector>
#include <chrono>

DecentralizedMultiAgentRRT::DecentralizedMultiAgentRRT(/* Add any necessary constructor parameters */) {
    // Initialize your planner or any other necessary components
}

amp::MultiAgentPath2D DecentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    // Timing variables
    auto start_time = std::chrono::high_resolution_clock::now();

    // Array to store paths for each agent
    std::vector<amp::Path2D> paths;

    // Iterate over each start/goal pair in priority order
    for (const auto& agent_pair : problem.getPriorityOrder()) {
        // Get the agent index
        size_t agent_index = agent_pair.first;

        // Get the start and goal configurations for the current agent
        amp::State2D start_state = problem.getStartState(agent_index);
        amp::State2D goal_state = problem.getGoalState(agent_index);

        // Create a MultiAgentRRT planner for the current agent
        MultiAgentRRT planner(/* Add any necessary parameters */);

        // Create a planning problem for the current agent
        amp::MultiAgentProblem2D single_agent_problem(/* Add any necessary parameters */);

        // Use the MultiAgentRRT planner to solve the problem for the current agent
        amp::Path2D single_agent_path = planner.plan(single_agent_problem);

        // Store the path for the current agent
        paths.push_back(single_agent_path);
    }

    // Calculate computation time
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double computation_time = duration.count() / 1e6; // Convert to seconds

    // Return the multi-agent paths and computation time
    return amp::MultiAgentPath2D(paths, computation_time);
}

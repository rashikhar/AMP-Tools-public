#include "MultiAgentRRT.h"
#include "AMPCore.h"
#include"hw/HW2.h"
#include"hw/HW5.h"
#include"hw/HW8.h"
#include"Eigen/Geometry"
#include"Q1.h"
#include <iostream>

MultiAgentRRT::MultiAgentRRT(double maxIterations, double stepSize, double goalBias, double epsilon)
    : maxIterations(maxIterations), stepSize(stepSize), goalBias(goalBias), epsilon(epsilon) {
}

amp::MultiAgentPath2D MultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    auto start_time = std::chrono::high_resolution_clock::now();

    tree.clear();
    std::vector<double> startState;
    for (const auto& robots : problem.robots) {
        startState.insert(end(startState), begin(robots.qStart), end(robots.qStart));
    }
    tree.push_back(new Node(startState, nullptr));

    for (int i = 0; i < maxIterations; ++i) {
        std::vector<double> randomState;
        for (const auto& robots : problem.robots) {
            if ((double)rand() / RAND_MAX < goalBias) {
                randomState.insert(end(randomState), begin(robots.qGoal), end(robots.qGoal));
            } else {
                randomState.emplace_back((double)rand() / RAND_MAX * (robots.xmax - robots.xmin) + robots.xmin);
                randomState.emplace_back((double)rand() / RAND_MAX * (robots.ymax - robots.ymin) + robots.ymin);
            }
        }

        Node* randomNode = new Node(randomState, nullptr);
        Node* nearestNode = getNearestNode(problem, *randomNode);
        Node* newNode = extend(problem, *nearestNode, *randomNode);

        if (isCollisionFree(problem, *nearestNode, *newNode)) {
            tree.push_back(newNode);

            if (isGoalReached(problem, *newNode)) {
                break;
            }
        } else {
            delete newNode;
        }
    }

    amp::MultiAgentPath2D path;
    path.size = tree.size();
    if (tree.back()->x == problem.robots.back().qGoal) {
        Node* current = tree.back();
        while (current != nullptr) {
            std::vector<double> state(current->x.begin(), current->x.begin() + problem.robots[0].qStart.size());
            path.paths.insert(begin(path.paths), state);
            current = current->parent;
        }
        std::reverse(path.paths.begin(), path.paths.end());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "Computation Time: " << duration.count() << " microseconds" << std::endl;

    // Clean up allocated memory
    for (auto node : tree) {
        delete node;
    }

    return path;
}

// *****
// bool MultiAgentRRT::isCollisionFree(const amp::MultiAgentProblem2D& problem, const Node& a, const Node& b) {
//     // Implement collision checking logic based on the problem's obstacles
//     // For simplicity I am going to assume collision-free for checking
//     return true;
// }
// *****

bool MultiAgentRRT::isCollisionFree(const MultiAgentProblem2D& problem, const std::vector<double>& state) {
    for (size_t i = 0; i < problem.robots.size(); ++i) {
        if (!isCollisionFreeForRobot(problem, state, i)) {
            return false;
        }
    }
    return true;
}

bool MultiAgentRRT::isCollisionFreeForRobot(const MultiAgentProblem2D& problem, const std::vector<double>& state, size_t robotIndex) {
    // Implement collision checking logic for a single robot at the given state
    double x = state[robotIndex * 2];
    double y = state[robotIndex * 2 + 1];

    for (const auto& obstacle : obstacles) {
        if (obstacle.isCollision(x, y)) {
            return false;  // Collision with obstacle
        }
    }


    return true;  // No collision
}

bool MultiAgentRRT::isExtendedLineCollisionFree(const MultiAgentProblem2D& problem, const std::vector<double>& from, const std::vector<double>& target) {
    for (size_t i = 0; i < problem.robots.size(); ++i) {
        if (!isExtendedLineCollisionFreeForRobot(problem, from, target, i)) {
            return false;
        }
    }
    return true;
}

bool MultiAgentRRT::isExtendedLineCollisionFreeForRobot(const MultiAgentProblem2D& problem, const std::vector<double>& from, const std::vector<double>& target, size_t robotIndex) {
    // Implement collision checking along the extended line for a single robot
    double dist = std::hypot(target[robotIndex * 2] - from[robotIndex * 2], target[robotIndex * 2 + 1] - from[robotIndex * 2 + 1]);
    int numSteps = static_cast<int>(dist / stepSize);

    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / numSteps;
        std::vector<double> state;

        for (size_t j = 0; j < problem.robots.size(); ++j) {
            double x = from[j * 2] + t * (target[j * 2] - from[j * 2]);
            double y = from[j * 2 + 1] + t * (target[j * 2 + 1] - from[j * 2 + 1]);
            state.push_back(x);
            state.push_back(y);
        }
        if (!isCollisionFreeForRobot(problem, state, robotIndex)) {
            return false;  // Collision along the extended line
        }
    }

    return true;  // No collision along the extended line
}   


MultiAgentRRT::Node* MultiAgentRRT::getNearestNode(const amp::MultiAgentProblem2D& problem, const Node& target) {
    Node* nearest = tree[0];
    double minDist = 0.0;

    for (const auto& node : tree) {
        double dist = 0.0;
        for (size_t i = 0; i < node->x.size(); ++i) {
            dist += std::pow(node->x[i] - target.x[i], 2);
        }
        dist = std::sqrt(dist);

        if (dist < minDist || nearest == tree[0]) {
            nearest = node;
            minDist = dist;
        }
    }

    return nearest;
}

MultiAgentRRT::Node* MultiAgentRRT::extend(const amp::MultiAgentProblem2D& problem, const Node& from, const Node& target) {
    std::vector<double> newState;

    for (size_t i = 0; i < from.x.size(); ++i) {
        double dist = target.x[i] - from.x[i];
        double delta = std::min(stepSize, std::abs(dist));
        newState.push_back(from.x[i] + delta * (dist >= 0 ? 1 : -1));
    }

    return new Node(newState, &const_cast<Node&>(from));
}

bool MultiAgentRRT::isGoalReached(const amp::MultiAgentProblem2D& problem, const Node& node) {
    for (size_t i = 0; i < problem.robots.size(); ++i) {
        double distToGoal = std::hypot(node.x[i * 2] - problem.robots[i].qGoal[0], node.x[i * 2 + 1] - problem.robots[i].qGoal[1]);
        if (distToGoal > epsilon) {
            return false;
        }
    }

    return true;
}

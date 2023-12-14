#pragma once

#include <iostream>
#include "tools/Algorithms.h"
#include "tools/Environment.h"
#include "tools/Path.h"
#include "tools/Graph.h"
#include"hw/HW8.h"
#include"Eigen/Geometry"
#include"HelpfulClass.h"
#include "AMPCore.h"
#include "MyRRT.h"

// void MyRRT::setup(double start[2], double goal[2], double minDistance) {
//     // My other code...
// }

// void MyRRT::step(double stepSize, double maximumSteps) {
//     // My other code...
// }

// bool MyRRT::reach(double target[2], double maximumSteps) {
//     // My other code...
// }

// bool MyRRT::checkCollision(double from[2], double to[2]) {
//     // My other code...
// }


MyRRT2D::MyRRT2D(double maxIterations, double stepSize, double goalBias, double epsilon)
    : maxIterations(maxIterations), stepSize(stepSize), goalBias(goalBias), epsilon(epsilon) {
}

amp::Path2D MyRRT2D::plan(const amp::Problem2D& problem) {
    auto start_time = std::chrono::high_resolution_clock::now();

    tree.clear();
    tree.push_back(new Node(problem.qStart.x, problem.qStart.y, nullptr));

    for (int i = 0; i < maxIterations; ++i) {
        Node* randomNode;
        if ((double)rand() / RAND_MAX < goalBias) {
            randomNode = new Node(problem.qGoal.x, problem.qGoal.y, nullptr);
        } else {
            randomNode = new Node((double)rand() / RAND_MAX * (problem.xmax - problem.xmin) + problem.xmin,
                                  (double)rand() / RAND_MAX * (problem.ymax - problem.ymin) + problem.ymin, nullptr);
        }

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

    amp::Path2D path;
    if (tree.back()->x == problem.qGoal.x && tree.back()->y == problem.qGoal.y) {
        Node* current = tree.back();
        while (current != nullptr) {
            path.waypoints.emplace_back(current->x, current->y);
            current = current->parent;
        }
        std::reverse(path.waypoints.begin(), path.waypoints.end());
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

bool MyRRT2D::isCollisionFree(const amp::Problem2D& problem, const Node& a, const Node& b) {
    // Implement collision checking logic based on the problem's obstacles
    // For simplicity, assuming collision-free for demonstration
    return true;
}

MyRRT2D::Node* MyRRT2D::getNearestNode(const amp::Problem2D& problem, const Node& target) {
    Node* nearest = tree[0];
    double minDist = std::hypot(nearest->x - target.x, nearest->y - target.y);

    for (const auto& node : tree) {
        double dist = std::hypot(node->x - target.x, node->y - target.y);
        if (dist < minDist) {
            nearest = node;
            minDist = dist;
        }
    }

    return nearest;
}

MyRRT2D::Node* MyRRT2D::extend(const amp::Problem2D& problem, const Node& from, const Node& target) {
    double dist = std::hypot(from.x - target.x, from.y - target.y);
    if (dist < stepSize) {
        return new Node(target.x, target.y, &const_cast<Node&>(from));
    } else {
        double theta = std::atan2(target.y - from.y, target.x - from.x);
        double newX = from.x + stepSize * std::cos(theta);
        double newY = from.y + stepSize * std::sin(theta);

        return new Node(newX, newY, &const_cast<Node&>(from));
    }
}

bool MyRRT2D::isGoalReached(const amp::Problem2D& problem, const Node& node) {
    // Check if the node is close to the goal within epsilon
    double distToGoal = std::hypot(node.x - problem.qGoal.x, node.y - problem.qGoal.y);
    return distToGoal < epsilon;
}

// // rrt.h

// #pragma once

// #include "tools/Algorithms.h"
// #include "tools/Environment.h"
// #include "tools/Path.h"
// #include "tools/Graph.h"
// #include "hw/HW7.h"
// #include "Eigen/Geometry"
// #include "HelpfulClass.h"
// #include "Obstacle.h"
// #include "AMPCore.h"
// #include <Eigen/Dense>
// #include <vector>
// #include <random>
// #include <cmath>

// class MyRRT2D {
// private:
//     std::vector<Eigen::Vector2d> nodes;
//     double stepSize;
//     double goalBias;
//     double epsilon; // For termination condition
//     Eigen::Vector2d goal;

// public:
//     MyRRT2D(const Eigen::Vector2d& bound_x, const Eigen::Vector2d& bound_y, double n, double r, double p_goal, double e)
//         : stepSize(r), goalBias(p_goal), epsilon(e) {

//             // Consider adding the start point as the first node if needed
//     }

//     Eigen::Vector2d getRandomPoint(const Eigen::Vector2d& bound_x, const Eigen::Vector2d& bound_y) {
//         // Add logic to get a random point within the given bounds
//         Eigen::Vector2d getRandomPoint(const Eigen::Vector2d& bound_x, const Eigen::Vector2d& bound_y) {
//         static std::random_device rd;
//         static std::mt19937 gen(rd());
//         std::uniform_real_distribution<double> disX(bound_x.x(), bound_x.y());
//         std::uniform_real_distribution<double> disY(bound_y.x(), bound_y.y());
//         return { disX(gen), disY(gen) };
//         }
//     }

//     size_t findNearestNode(const Eigen::Vector2d& point) {

//     // Add logic to find the nearest node in the tree to a given point
//     size_t findNearestNode(const Eigen::Vector2d& point) {
//     double minDist = std::numeric_limits<double>::max();
//     size_t nearestIdx = 0;

//     for (size_t i = 0; i < nodes.size(); ++i) {
//         double dist = (point - nodes[i]).norm();
//         if (dist < minDist) {
//             minDist = dist;
//             nearestIdx = i;
//         }
//     }
//     return nearestIdx;
// }

//     }

//     Eigen::Vector2d steer(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
//         // Add logic to steer from 'from' point towards 'to' point
//         Eigen::Vector2d steer(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
//     double dist = (to - from).norm();
//     if (dist <= stepSize) {
//         return to;
//     } else {
//         Eigen::Vector2d direction = (to - from).normalized();
//         return from + direction * stepSize;
//     }
// }

//     }

//     Eigen::Vector2d randomPointWithGoalBias(const Eigen::Vector2d& qgoal) {
//         // Add logic to generate a random point biased towards the goal
//         // You will use 'goalBias' to bias the sampling towards the goal
//         Eigen::Vector2d randomPointWithGoalBias(const Eigen::Vector2d& qgoal) {
//         static std::random_device rd;
//         static std::mt19937 gen(rd());
//         std::uniform_real_distribution<double> dis(0.0, 1.0);
//         double sample = dis(gen);

//         if (sample < goalBias) {
//             return qgoal; // Bias towards the goal
//         } else {
//             return getRandomPoint(Eigen::Vector2d(-7, 35), Eigen::Vector2d(-7, 7)); // Modify range according to your C-space
//         }
//     }

//     }

//     amp::Path2D plan(const amp::Problem2D& problem) {
//         // Add your planning logic here
//         // Implement the RRT algorithm to plan the path based on the given problem
//         // This is a placeholder and needs to be filled with the RRT planning logic
//         amp::Path2D plan(const amp::Problem2D& problem) {
//         amp::Path2D path;
//         // Assuming 'start' is the start point of the problem
//         nodes.push_back(start); // Add the start point as the first node

//         for (int i = 0; i < n; ++i) {
//             Eigen::Vector2d randomPoint = getRandomPoint(bound_x, bound_y);
//             size_t nearestIdx = findNearestNode(randomPoint);
//             Eigen::Vector2d nearestNode = nodes[nearestIdx];
//             Eigen::Vector2d newPoint = steer(nearestNode, randomPoint);
//             if (isCollisionFree(newPoint, problem.obstacles)) {
//                 nodes.push_back(newPoint);
//                 if (solutionFound(newPoint, goal, epsilon)) {
//                     // Generate path from the found solution to start
//                     path.waypoints.push_back(newPoint);
//                     size_t idx = nodes.size() - 1;
//                     while (idx > 0) {
//                         path.waypoints.push_back(nodes[idx]);
//                         idx = idx; // Index of the parent node
//                     }
//                     std::reverse(path.waypoints.begin(), path.waypoints.end());
//                     break; // Solution found, exit loop
//                 }
//             }
// }
//         return path;
//     }
//         // amp::Path2D path; // This path is empty, you should fill it with the planned path
//     }

//  std::tuple<bool, double, double> planCompare(const amp::Problem2D& problem) {
//         // Implement comparison logic between paths and their computation times
//         // Run RRT for 100 iterations, calculate average path length and computation time
//         double avgPathLength = 0.0; // Calculate the average path length
//         double avgComputationTime = 0.0; // Calculate the average computation time
//         bool success = false; // Check if a path is found
//         double totalPathLength = 0.0;
//         double totalComputationTime = 0.0;
//         int successCount = 0;

//         for (int i = 0; i < 100; ++i) {
//             amp::Path2D path = plan(problem);
//             if (!path.waypoints.empty()) {
//                 totalPathLength += path.length();
//                 totalComputationTime += /* Calculate the time taken for planning */;
//                 successCount++;
//             }
//         }

//         double avgPathLength = successCount > 0 ? totalPathLength / successCount : 0.0;
//         double avgComputationTime = successCount > 0 ? totalComputationTime / successCount : 0.0;

//         return std::make_tuple(successCount > 0, avgPathLength, avgComputationTime);
//     }
//     }

//     bool isCollisionFree(const Eigen::Vector2d& point, const std::vector<amp::Obstacle2D>& obstacles) {
//         // Collision checking logic
//     for (const auto& obstacle : obstacles) {
//         const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
//         size_t numVertices = vertices.size();

//         bool collision = false;
//         for (size_t i = 0, j = numVertices - 1; i < numVertices; j = i++) {
//             const Eigen::Vector2d& vi = vertices[i];
//             const Eigen::Vector2d& vj = vertices[j];
//             if (((vi.y() > point.y()) != (vj.y() > point.y())) &&
//                 (point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) / (vj.y() - vi.y()) + vi.x())) {
//                 collision = !collision;
//             }
//         }
//         if (collision) {
//             return false; // Collision detected with at least one obstacle
//         }
//     }
//     return true; // No collision with any obstacle
//     }

//     bool solutionFound(const Eigen::Vector2d& currentPoint, const Eigen::Vector2d& goalPoint, double epsilon) {
//         // Solution found logic
//         // Calculate the Euclidean distance between currentPoint and goalPoint
//     double distance = (currentPoint - goalPoint).norm();

//     // Check if the distance is within the specified epsilon
//     return (distance <= epsilon);x
//     };


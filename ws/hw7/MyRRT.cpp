// #pragma once
// #include "tools/Algorithms.h"
// #include "tools/Environment.h"
// #include "tools/Path.h"
// #include "tools/Graph.h"
// #include"hw/HW7.h"
// #include"Eigen/Geometry"
// #include"HelpfulClass.h"
// #include "AMPCore.h"
// #include "MyRRT.h"

// // void MyRRT::setup(double start[2], double goal[2], double minDistance) {
// //     // Your other code...
// // }

// // void MyRRT::step(double stepSize, double maximumSteps) {
// //     // Your other code...
// // }

// // bool MyRRT::reach(double target[2], double maximumSteps) {
// //     // Your other code...
// // }

// // bool MyRRT::checkCollision(double from[2], double to[2]) {
// //     // Your other code...
// // }


// MyRRT2D::MyRRT2D(double n, double r, double p_goal, double epsilon) :
//     n(n),
//     r(r),
//     p_goal(p_goal),
//     epsilon(epsilon) {}

// MyRRT2D::~MyRRT2D() {}

// double MyRRT2D::getDistance(const amp::Vector2d& point1, const amp::Vector2d& point2) {
//     return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
// }

// double MyRRT2D::getDistance(const amp::Node2D& node1, const amp::Node2D& node2) {
//     return getDistance(node1.position, node2.position);
// }

// double MyRRT2D::getRandomDistance(std::mt19937& rng) {
//     std::uniform_real_distribution<double> unif(0, 1);
//     double r = unif(rng);
//     return r * r * n;
// }

// bool MyRRT2D::checkCollision(const amp::Node2D& node, const amp::Problem2D& problem) {
//     for (const auto& obstacle : problem.obstacles) {
//         if (obstacle.isInside(node.position)) {
//             return true;
//         }
//     }
//     return false;
// }

// amp::Node2D MyRRT2D::steer(const amp::Node2D& fromNode, const amp::Node2D& toNode, const amp::Problem2D& problem) {
//     double distance = getDistance(fromNode, toNode);
//     amp::Vector2d newPosition = fromNode.position + distance * (toNode.position - fromNode.position).normalized();
//     amp::Node2D newNode(newPosition);
//     return newNode;
// }

// bool MyRRT2D::reachGoal(const amp::Node2D& node, const amp::Problem2D& problem) {
//     return getDistance(node, problem.qgoal) <= epsilon;
// }

// amp::Path2D MyRRT2D::plan(const amp::Problem2D& problem) {
//     std::mt19937 rng(time(0));
//     std::uniform_real_distribution<double> unif(0, 1);

//     amp::Node2D startNode(problem.qstart);
//     startNode.parent = &startNode;

//     std::vector<amp::Node2D> nodes = {startNode};

//     while (true) {
//         double maxDistance = std::numeric_limits<double>::max();
//         amp::Node2D* nearestNode = nullptr;
//         for (const auto& node : nodes) {
//             double distance = getRandomDistance(rng);
//             if (distance < maxDistance) {
//                 maxDistance = distance;
//                 nearestNode = &node;
//             }
//         }

//         if (unif(rng) < p_goal) {
//             amp::Node2D goalNode(problem.qgoal);
//             double distance = getDistance(*nearestNode, goalNode);
//             if (distance <= maxDistance && !checkCollision(goalNode, problem)) {
//                 return amp::Path2D(startNode, goalNode);
//             }
//         }

//         amp::Vector2d newPosition = nearestNode->position + maxDistance * amp::Vector2d(unif(rng) - 0.5, unif(rng) - 0.5).normalized();
//         amp::Node2D newNode(newPosition);

//         if (!checkCollision(newNode, problem)) {
//             nodes.push_back(newNode);
//             newNode.parent = nearestNode;
//         }
//     }
// }

#pragma once
#include "tools/Algorithms.h"
#include "tools/Environment.h"
#include "tools/Path.h"
#include "tools/Graph.h"
#include"hw/HW7.h"
#include"Eigen/Geometry"
#include"HelpfulClass.h"
#include "AMPCore.h"

#include <tuple>


// class MyPRM2D : public PointMotionPlanner2D {
// private:
//     Eigen::Vector2d bound_x;
//     Eigen::Vector2d bound_y;
//     int n;
//     double r;

// public:
//     MyPRM2D(const Eigen::Vector2d& xBound, const Eigen::Vector2d& yBound, int numNodes, double radius)
//         : bound_x(xBound), bound_y(yBound), n(numNodes), r(radius) {}

//     amp::Path2D plan(const amp::Problem2D& problem) override {
//         amp::Path2D path;
//         // Implement your PRM planning algorithm here
//         std::vector<Eigen::Vector2d> roadmap;
//         generateRoadmap(roadmap);

//         // Connect roadmap nodes
//         connectRoadmap(roadmap, problem);

//         // Find a path using graph search or other algorithms
//         path = findPath(roadmap, problem);
//         return path;
//     }
// // private:
//     // Functions for PRM planning logic

//     void generateRoadmap(std::vector<Eigen::Vector2d>& roadmap) {
//         // Generate random nodes within the boundaries
//         for (int i = 0; i < n; ++i) {
//             double x = /* generate random x within bounds */;
//             double y = /* generate random y within bounds */;
//             roadmap.push_back(Eigen::Vector2d(x, y));
//         }
//     }

// void connectRoadmap(const std::vector<Eigen::Vector2d>& roadmap, const amp::Problem2D& problem) {
//         // Connect nodes based on distances and collision checking
//         for (size_t i = 0; i < roadmap.size(); ++i) {
//             for (size_t j = i + 1; j < roadmap.size(); ++j) {
//                 // Check if nodes are within distance and there's no collision
//                 if (distance(roadmap[i], roadmap[j]) <= r && !checkCollision(roadmap[i], roadmap[j], problem)) {
//                     // Connect nodes (add edges)
                    

//                 }
//             }
//         }
//     }

//     double distance(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
//         // Calculate Euclidean distance between two points
//         return (v2 - v1).norm();
//     }

//         // Placeholder for obtaining a path

//         return path;
//     };
//     bool checkCollision(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const amp::Problem2D& problem) {
//         // Check for collision between nodes v1 and v2 with the given problem
//         // ...
//         // Return true if there's a collision, false otherwise
//         return /* collision check logic */;
//     }

//     amp::Path2D findPath(const std::vector<Eigen::Vector2d>& roadmap, const amp::Problem2D& problem) {
//         amp::Path2D path;

//         // Implement a path-finding algorithm using the roadmap


//         return path;
//     };

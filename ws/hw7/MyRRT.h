// #pragma once
// #include <iostream>
// #include <random>
// #include <ctime>
// #include <cmath>
// #include <limits>
// #include "tools/Algorithms.h"
// #include "tools/Environment.h"
// #include "tools/Path.h"
// #include "tools/Graph.h"
// #include"hw/HW7.h"
// #include"Eigen/Geometry"
// #include"HelpfulClass.h"
// #include "AMPCore.h"


// // class MyRRT {
// //     // Your other code...

// //     public:
// //         void setup(double start[2], double goal[2], double minDistance);
// //         void step(double stepSize, double maximumSteps);
// //         bool reach(double target[2], double maximumSteps);
// //         bool checkCollision(double from[2], double to[2]);
// // };


// class MyRRT2D {
//     public:
//         MyRRT2D(double n, double r, double p_goal, double epsilon);
//         virtual ~MyRRT2D();
//         virtual amp::Path2D plan(const amp::Problem2D& problem);

//     private:
//         // parameters
//         double n;
//         double r;
//         double p_goal;
//         double epsilon;

//         // private functions
//         double getDistance(const amp::Vector2d& point1, const amp::Vector2d& point2);
//         double getDistance(const amp::Node2D& node1, const amp::Node2D& node2);
//         double getRandomDistance(std::mt19937& rng);
//         bool checkCollision(const amp::Node2D& node, const amp::Problem2D& problem);
//         amp::Node2D steer(const amp::Node2D& fromNode, const amp::Node2D& toNode, const amp::Problem2D& problem);
//         bool reachGoal(const amp::Node2D& node, const amp::Problem2D& problem);
// };
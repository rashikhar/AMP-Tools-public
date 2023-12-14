#pragma once
#include <iostream>
#include <random>
#include <ctime>
#include <cmath>
#include <limits>
#include <amp/Path2D.h>
#include <amp/Problem2D.h>
#include <amp/Visualizer.h>
#include "tools/Algorithms.h"
#include "tools/Environment.h"
#include "tools/Path.h"
#include "tools/Graph.h"
#include"hw/HW8.h"
#include"Eigen/Geometry"
#include"HelpfulClass.h"
#include "AMPCore.h"


// class MyRRT {
//     // Your other code...

//     public:
//         void setup(double start[2], double goal[2], double minDistance);
//         void step(double stepSize, double maximumSteps);
//         bool reach(double target[2], double maximumSteps);
//         bool checkCollision(double from[2], double to[2]);
// };

#include "Problem2D.h"
#include "Path2D.h"
#include "Visualizer.h"
#include <vector>
#include <chrono>

class MyRRT2D {
public:
    MyRRT2D(double maxIterations, double stepSize, double goalBias, double epsilon);
    amp::Path2D plan(const amp::Problem2D& problem);

private:
    double maxIterations;
    double stepSize;
    double goalBias;
    double epsilon;

    struct Node {
        double x, y;
        Node* parent;

        Node(double x, double y, Node* parent) : x(x), y(y), parent(parent) {}
    };

    std::vector<Node*> tree;

    bool isCollisionFree(const amp::Problem2D& problem, const Node& a, const Node& b);
    Node* getNearestNode(const amp::Problem2D& problem, const Node& target);
    Node* extend(const amp::Problem2D& problem, const Node& from, const Node& target);
    bool isGoalReached(const amp::Problem2D& problem, const Node& node);
};



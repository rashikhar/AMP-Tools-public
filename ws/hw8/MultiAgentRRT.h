#pragma once

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

class MultiAgentRRT {
public:
    MultiAgentRRT(double maxIterations, double stepSize, double goalBias, double epsilon);
    amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem);

private:
    double maxIterations;
    double stepSize;
    double goalBias;
    double epsilon;

    struct Node {
        std::vector<double> x;
        Node* parent;

        Node(const std::vector<double>& x, Node* parent) : x(x), parent(parent) {}
    };

    std::vector<Node*> tree;

    bool isCollisionFree(const MultiAgentProblem2D& problem, const std::vector<double>& state);
    bool isExtendedLineCollisionFree(const MultiAgentProblem2D& problem, const std::vector<double>& from, const std::vector<double>& target);
    Node* getNearestNode(const amp::MultiAgentProblem2D& problem, const Node& target);
    Node* extend(const amp::MultiAgentProblem2D& problem, const Node& from, const Node& target);
    bool isGoalReached(const amp::MultiAgentProblem2D& problem, const Node& node);
};


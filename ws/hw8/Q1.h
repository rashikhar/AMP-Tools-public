#pragma once
#include "tools/Algorithms.h"
#include "tools/Environment.h"
#include "tools/Path.h"
#include "tools/Graph.h"
#include"hw/HW8.h"
#include"Eigen/Geometry"
#include"HelpfulClass.h"
#include "AMPCore.h"

#include <tuple>

class MyCRRT : public amp::CentralizedMultiAgentRRT{
    public:
    MyCRRT(double n, double r, double p, double epsilon): n(n), r(r), p(p){}
virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

private:
    int n, m;
    double r, p;
    std::vector<double> radii;
};


class CentralizedMultiAgentRRT : public amp::MultiAgentCircleMotionPlanner2D {
public:
    amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override {
        // My implementation of the plan method goes here
        // Make sure to return a value of type amp::MultiAgentPath2D as expected
        // Implement your logic or algorithm for path planning based on the problem passed
    }
    // Other methods and members specific to the derived class...
};
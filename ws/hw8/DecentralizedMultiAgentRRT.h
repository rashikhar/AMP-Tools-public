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

class PriorityDecentralizedMultiAgentRRT : public DecentralizedMultiAgentRRT {
public:
    PriorityDecentralizedMultiAgentRRT(/* constructor parameters */);

    amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

private:
    // Add any private members or helper methods as needed
};


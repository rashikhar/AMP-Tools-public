#include "AMPCore.h"
#include"hw/HW2.h"
#include"hw/HW5.h"
#include"hw/HW8.h"
#include"Eigen/Geometry"
#include"Q1.h"
#include "MultiAgentRRT.h"
#include "Visualizer.h"

using namespace Eigen;
using namespace amp;

void Problem1(){
// Get the workspace for m = 2 robots
// amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1();
// MyCRRT planner ()

// Set the parameters
double n = 7500;
double r = 0.5;
double p_goal= 0.05;
double epsilon = 0.25;

// Get the workspace for m = 2 robots
amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1();
// MyCRRT planner(n, r, p_goal, epsilon);
// amp::MultiAgentPath2D path = planner.plan(problem);

// Create MultiAgentRRT
MultiAgentRRT planner(n, r, p_goal, epsilon);

// Plan using MultiAgentRRT
amp::MultiAgentPath2D path = planner.plan(problem);

Visualizer::makeFigure(problem, path);
Visualizer::showFigures();
}

// Exercise 2
void Problem2(){
// Get the workspace for m = 2 robots
// amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1();
// MyCRRT planner ()

// Set the parameters
double n = 7500;
double r = 0.5;
double p_goal= 0.05;
double epsilon = 0.25;

// Get the workspace for m = 2 robots
amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1();
// MyCRRT planner(n, r, p_goal, epsilon);
// amp::MultiAgentPath2D path = planner.plan(problem);

// Create MultiAgentRRT
DecentralizedMultiAgentRRT planner(n, r, p_goal, epsilon);

// Plan using MultiAgentRRT
amp::MultiAgentPath2D path = planner.plan(problem);

Visualizer::makeFigure(problem, path);
Visualizer::showFigures();
}



int main(int argc, char** argv) {
     // Problem1a();
    Problem1();
    // Problem2a();
    //Problem2b();
    //Problem3();
    amp::Visualizer::showFigures();


    // Grade method
    // amp::HW4::grade<LinkManipulator2D>(constructor, "rashikha.jagula@colorado.edu", argc, argv);

    return 0;
}
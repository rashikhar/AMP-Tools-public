// // // #include "AMPCore.h"
// // // #include"hw/HW2.h"
// // // #include"hw/HW5.h"
// // // #include"hw/HW7.h"
// // // #include"Eigen/Geometry"
// // // #include"HelpfulClass.h"
// // // #include"prm.h"
// // // #include"rrt.h"

 int main(int argc, char** argv) {

// // // // Which cases to run
// // // struct cases {
// // // bool log;
// // // std::string name;
// // // };
// // // std::vector<cases> options = {
// // //     {false, "Exercise 1a.1"},
// // //     {false, "Exercise 1a.2"},
// // //     {false, "Exercise 1a.4"},
// // //     {false, "Exercise 1b.1"},
// // //     {false, "Exercise 1b.2"},
// // //     {false, "Exercise 1b.4"},
// // //     {false, "Exercise 2a"},
// // //     {false, "Exercise 2b"}
// // // };

// // // //***Exercise 1 *** PRM***//
// // // // 1a.1

// // //     if (options[0].log){

// // // // Get problem, Hw5
// //     // 

// // // // Boundaries definition
// // //     Eigen::Vector2d bound_x = Eigen::Vector2d(-1, 11);
// // //     Eigen::Vector2d bound_y = Eigen::Vector2d(-3, 3);

// // // // Initial parameters
// // //     int n = 200;
// // //     double r =1;

// // // // Creating an instance of PRM
// // // MyPRM2D prm(bound_x, bound_y, n, r);

// // // // Planing using the plan
// // // // amp::Path2D plan(const amp::Problem2D& problem) = 0;
// // // amp::Path2D path = prm.plan(problem);

// // // // logging path length
// // // if (path.waypoints.size() > 0){
// // // std::cout << "1a.1. Path length: " << path.length() << std::endl;
// // // }
// // // else{
// // // std::cout << "1a.1. Path length: " << "No path found" << std::endl;
// // // }
// // // // Visualize
// // // // amp::HW7::hint();
// // // amp::Visualizer::makeFigure(problem, path);
// // //     }

// // // //***Exercise 1 *** PRM***//
// // // // 1a.2
// // // if (options[1].log){
// // // std::list<std::vector<double>> data_set = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
// // // std::vector<std::string> labels = {"n=200, 1=0.5", "n=200, r=1", "n=200, r=1.5", "n=200", "r=2", "n=500", "r=0.5", "n=500", "r=1", "n=500", "r=1.5", "n=500", "r=2"};
// // // std::list<std::vector<double>> success_list; 
// // // std::list<std::vector<double>> pathlength_list;
// // // std::list<std::vector<double>> computationTime_list;

// // // for (auto currentNode : data_set){
// // // // Get problem, Hw5
// // // amp::Problem2D problem = amp::HW5::getWorkspace1();

// // // // Boundaries definition
// // // Eigen::Vector2d bound_x = Eigen::Vector2d(-1, 11);
// // // Eigen::Vector2d bound_y = Eigen::Vector2d(-3, 3);

// // // double success_amount = 0;

// // // // Initial parameters
// // // double n = currentNode[0];
// // // double r = currentNode[1];

// // // // output the parameters
// // // std::cout << "n: " << n << "r: " << r << std::endl;

// // // // create vectors to store the path length and computation time for each 100 iteration loop 
// // // std::vector<double> pathLength_vec;
// // // std::vector<double> computationTime_vec;

// // // // run 100 iterations
// // // for (int i = 0; i<100; i++){

// // // // Create PRM
// // // MyPRM2D prm(bound_x, bound_y, n, r);

// // // // Get the result of the planCompare function
// // // std::tuple<bool, double, double, amp::Path2D> result = prm.planCompare(problem);

// // // amp::Visualizer::makeFigure(problem, std::get<3>(result));

// // // //***Exercise 1 *** PRM***//
// // // // 1a.4
// // // // Path smoothing for benchmark
// // //     amp:: Path2D newPath;
// // //     if(options[2].log && std::get<0>(result)){
// // //     newPath = pathSmooth(std::get<3>(result), problem);
// // //     }

// // //     // Add to the success vector
// // //     if(std::get<0>(result)){
// // //     success_amount = success_amount + 1;
// // //     }

// // //     // Add the path length vector, only if its non zero 
// // //     if(options[2].log && std::get<0>(result)){
// // //     pathLength_vec.push_back(newPath.length());
// // //     }else{
// // //     if (std::get<1>(result) > 0){
// // //     pathLength_vec.push_back(std::get<1>(result));
// // //     }
// // //     }
// // //     // Add to the computation time vector, dividing by le6 to get into seconds 
// // //     computationTime_vec.push_back({std::get<2>(result)/1e3}) ;
// // // }
// // //     // Store in lists
// // //     success_list. push_back({success_amount}); 
// // //     pathLength_list.push_back(pathLength_vec);
// // //     computationTime_list.push_back(computationTime_vec);
// // //     }
// // // // Plot
// // //     amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 1a.2., HW5 workspace", "Number of Nodes, Neighborhood Radius", "Number of Successes");
// // //     amp::Visualizer::makeBoxPlot(pathlength_list, labels, "Excersie 1a.2., HW5 workspace", "Number of Nodes, Neighborhood Radius", "Path Length");
// // //     amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 1a.2., HW5 workspace", "Number of Nodes, Neighborhood Radius", "Computation Time (milliseconds)");
// // // }

// // // //***Exercise 1 *** PRM***//
// // // // 1b.1
// // // if (options[3].log) {
// // // // Get problem, Hw2, ws1
// // // amp::Problem2D problem = amp::HW2::getWorkspace1();

// // // // Bounds as defined
// // // Eigen::Vector2d bound_x = Eigen::Vector2d(problem.x_min, problem.x_max);
// // // Eigen::Vector2d bound_y = Eigen::Vector2d(problem.y_min, problem.y_max);

// // // // Initial parameters
// // // int n = 200;
// // // double r = 2;

// // // // Create PRM
// // // MyPRM2D prm(bound_x, bound_y, n, r);

// // // // Plan using • plan
// // // amp::Path2D path = prm.plan(problem);
// // // if (path.waypoints. size() > 0){
// // // std::cout << "1b.1 Path length: " << path.length() << std::endl;
// // // }
// // // else{
// // // std::cout << "1b.1 Path length: " << "No path found" << std::endl;
// // // }
// // // // Visualize
// // // amp::Visualizer::makeFigure(problem, path);

// // // // Get problem, Hw2, ws2
// // // amp::Problem2D problem2 = amp::HW2::getWorkspace2();

// // // // Bounds as defined
// // // Eigen::Vector2d bound_xb2 = Eigen::Vector2d(-7, 35);
// // // Eigen::Vector2d bound_yb2 = Eigen::Vector2d(-7, 7);

// // // // Initialize the parameters
// // // int nb2 = 200;
// // // double rb2 = 2;

// // // // Create PRM
// // // MyPRM2D prm2 (bound_xb2, bound_yb2, nb2, rb2);

// // // // Plan using •plan
// // // amp::Path2D path2 = prm2.plan(problem2);

// // // if (path2.waypoints.size() > 0){
// // // std::cout << "1b.1.ws2 Path length: " << path2.length << std::endl;
// // // }
// // // else{
// // // std::cout << "1b.1.ws2 Path length: " << "No path found" << std::endl;
// // // }
// // // // Visualize
// // // amp::Visualizer::makeFigure(problem2, path2);
// // // }

// // // //***Exercise 1 *** PRM***//
// // // // 1b.2
// // // if (options[4].log) {
// // // std::list<std::vector<double>> data_set = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000,1}, {1000,2}};
// // // std::vector<std::string> labels = {"n=200, r=1", "n=200, r=2", "n=500, 1=1", "n=500, r=2", "n=1000, r=1",  "n=1000, r=2",};
// // // std::list<std::vector<double>> success_list; 
// // // std::list<std::vector<double>> pathLength_list; 
// // // std::list<std::vector<double>> computationTime_list;

// // // // now loop through all data_set trials, running prm with the given parameters, storing the success rate
// // // for (auto currentNode : data_set){

// // // // / Get problem, Hw5
// // // amp::Problem2D problem = amp::HW2::getWorkspace2();

// // // // Boundaries definition
// // // Eigen::Vector2d bound_x = Eigen::Vector2d(problem.x_min, problem.x_max);
// // // Eigen::Vector2d bound_y = Eigen::Vector2d(problem.y_min, problem.y_max);
// // // double success_amount = 0;

// // // // Initial parameters
// // // double n = currentNode[0];
// // // double r = currentNode[1];

// // // // output the parameters
// // // std::cout << "n: " << n << " r: " << r << std::endl;

// // // // create vectors to store the path length and computation time for each 100 iteration loop
// // // std::vector<double> pathLength_vec;
// // // std::vector<double> computationTime_vec;

// // // // run 100 iterations
// // // for (int i = 0; i<100; i++){

// // // // Create PRM
// // // MyPRM2D prm(bound_x, bound_y, n, r);

// // // // Get the result of the planCompare function
// // // std::tuple<bool, double, double, amp::Path2D> result = prm.planCompare(problem);

// // // amp::Path2D newPath;
// // // if (options[5].log && std::get<0>(result)){
// // // newPath = pathSmooth(std::get<3>(result), problem);
// // // }

// // // // Add to the success vector
// // // if (std::get<0>(result)){
// // // success_amount = success_amount + 1;
// // // }

// // // // Add the path length vector, only if its non zero
// // // if (options[5].log && std::get<0>(result)){
// // // pathLength_vec.push_back(newPath.length());
// // // }else{
// // // if (std::get<1>(result) > 0){
// // // pathLength_vec. push_back(std::get<1>(result));
// // // }
// // // //  Add to the computation time vector, dividing by le6 to get into seconds 
// // // computationTime_vec.push_back({std::get<2>(result)/1e6});
// // // }
// // // // Store in lists
// // // success_list.push_back(success_amount); 
// // // pathLength_list.push_back(pathLength_vec); 
// // // computationTime_list.push_back(computationTime_vec);
// // // }

// // // // Plot
// // // amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 1b.2., HW2 Workspace 2", "Number of Nodes, Neighborhood Radius", "Number of Successes");
// // // amp::Visualizer::makeBoxPlot(pathLength_list, labels, "Excersie 1b.2., HW2 Workspace 2", "Number of Nodes, Neighborhood Radius", "Path Length");
// // // amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 1b.2., HW2 Workspace 2", "Number of Nodes, Neighborhood Radius", "Computation Time (milliseconds)");
// // // }

// // // //***Exercise 2 *** RRT***//
// // // // 2a
// // // if (options[6].log){

// // // // Initializing parameters
// // // double n = 5000;
// // // double r = 0.5;
// // // double p_goal = 0.05;
// // // double epsilon = 0.25;

// // // // Get problem, Hw5 2a
// // //  amp::Problem2D problem1 = amp::HW5::getWorkspace1();

// // // // Boundaries definition 
// // //  Eigen::Vector2d bound_x1 = Eigen::Vector2d(-1, 11);
// // //  Eigen::Vector2d bound_y1 = Eigen::Vector2d(-3, 3);

// // // // Create RRT
// // // MyRRT2D rrt1(bound_x1, bound_y1, n, r, p_goal, epsilon);

// // // // Plan using • plan
// // // amp::Path2D path1 = rrt1.plan(problem1);

// // // // Visualize
// // // amp::Visualizer::makeFigure (problem1, path1);

// // // // Output path length
// // // if (path1.waypoints.size() > 0){
// // //     std::cout << "2a. Path length: " << path1.length() << std::endl;
// // // }
// // // else{
// // //      std::cout << "2a. Path length: " << "No path found" << std::endl;
// // // }

// // // // Get problem, Hw2 ws1
// // // amp::Problem2D problem2 = amp::HW2::getWorkspace1();

// // // // Boundaries definition
// // // Eigen::Vector2d bound_x2 = Eigen::Vector2d(problem2.x_min, problem2.x_max);
// // // Eigen::Vector2d bound_y2 = Eigen::Vector2d(problem2.y_min, problem2.y_max);

// // // // Create RRT
// // // MyRRT2D rrt2(bound_x2, bound_y2, n, r, p_goal, epsilon);

// // // // Plan using • plan 
// // // amp::Path2D path2 = rrt2.plan(problem2);

// // // // Visualize
// // // amp::Visualizer::makeFigure(problem2, path2);

// // // // Output path length
// // // if (path2.waypoints.size () > 0){
// // //     std::cout << "2a. Path length: " << path2.length() << std::endl;
// // // else{
// // //     std::cout << "2a. Path length: " << "No path found" << std::endl;

// // // // Get problem, Hw2 ws2
// // //     amp::Problem2D problem3 = amp::HW2::getWorkspace2();

// // // // Bounds as defined
// // // Eigen::Vector2d bound_x3 = Eigen::Vector2d(-7, 35);
// // // Eigen::Vector2d bound_y3 = Eigen::Vector2d(-7, 7);

// // // // Create RRT
// // // MyRRT2D rrt3(bound_x3, bound_y3, n, r, p_goal, epsilon);

// // // // Plan using .plan
// // // amp::Path2D path3 = rrt3.plan(problem3);

// // // // Visualize
// // // amp::Visualizer::makeFigure(problem3, path3);

// // // // Output path length
// // // if (path3.waypoints.size() > 0){
// // // std::cout << "2a. Path length: " << path3.length() << std::endl;
// // // }
// // // else{
// // // sta::cout << "2a. Path length: " <<  "No path found" << std::endl;
// // // }
// // // }

// // // //***Exercise 2 *** RRT***//
// // // // 2b
// // // if (options[7].log){
// // // std::vector<amp::Problem2D> data_set = {
// // //     {amp::HW5::getWorkspace1(), 
// // //     amp::HW2: :getWorkspace1(), 
// // //     amp::HW2::getWorkspace2()};
// // // }
// // // std::vector<std::string> labels = {"HW5 Workspace 1", "HW2 Workspace 1", "HW2 Workspace 2"};
// // // std::list<std::vector<double>> success_list;
// // // std::list<std::vector<double>> pathLength_list;
// // // std::list<std::vector<double>> computationTime_list;

// // // // Initialize parameters
// // // double n = 5000;
// // // double r = 0.5;
// // // double p_goal = 0.05;
// // // double epsilon = 0.25;
// // // int i = 0;

// // // for (auto problem : data_set){
// // // i++;
// // // std::cout << "Running problem" << i << std::endl;
// // // Eigen::Vector2d bound_x;
// // // Eigen::Vector2d bound_y;

// // // if (i == 1) {
// // // // Boundaries definition
// // // bound_x = Eigen::Vector2d(-1, 11);
// // // bound_y = Eigen::Vector2d(-3, 3);
// // // }else{
// // // // Boundaries definition
// // // bound_x = Eigen::Vector2d(problem.x_min, problem.x_max);
// // // bound_y = Eigen::Vector2d(problem.y_min, problem.y_max);
// // // }

// // // // create vectors to store the path length and computation time for each 100 iteration 100p
// // // double success_amount = 0;
// // // std::vector<double> pathLength_vec;
// // // std::vector<double> computationTime_vec;

// // // // run 100 iterations
// // // for (int i = 0; i<100; i++){

// // // // Create PRM
// // //     MyRRT2D rrt(bound_x, bound_y, n, r, p_goal, epsilon);
    
// // // // Get the result of the planCompare function
// // //     std::tuple<bool, double, double> result = rrt.planCompare(problem);

// // // // Add to the success vector
// // // if (std::get<0>(result)){
// // // success_amount = success_amount + 1;
// // // }

// // // // Add the path length vector, only if its non zero
// // // if (std::get<1>(result) > 0){
// // // pathLength_vec.push_back({std::get<1>(result)});
// // // }

// // // // Add to the computation time vector, dividing by 1e6 to get into seconds 
// // // computationTime_vec.push_back({std::get<2>(result)/1e6});
// // // }

// // // // Store in lists
// // // success_list.push_back({success_amount}); 
// // // pathLength_list.push_back(pathLength_vec);
// // // computationTime_list.push_back(computationTime_vec);
// // // }

// // // // Plot
// // // amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 2.b., Success Comparison", "", "Number of Successes"); 
// // // amp::Visualizer::makeBoxPlot(pathLength_list, labels, "Excersie 2.b., Path Length Comparison", "", "Path Length"); 
// // // amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 2.b., Computation Time Comparison", "", "Computation Time (milliseconds)");
// // // }

// // // // Show plots
// // // amp::Visualizer::showFigures();

// // // // Grading 
// // // // amp::HW7::grade<MyPRM2D, MyRRT2D>("rashikha.jagula@colorado.edu", arge, argv);
// // // return 0;

// // //             }
// // //         }
// // //     }      
// // // }

// // int main(){
// //     return 0;
// // }

// #include <iostream>
// #include <random>
// #include <ctime>
// #include <amp/Path2D.h>
// #include <amp/Problem2D.h>
// #include <amp/Visualizer.h>
// #include "MyRRT2D.h"

// int main() {
//     // define the environment
//     amp::Problem2D problem;
//     problem.qstart = amp::Vector2d(0, 0);
//     problem.qgoal = amp::Vector2d(10, 10);
//     problem.obstacles.push_back(amp::Rectangle(amp::Vector2d(4, 4), 2, 2));

//     // create a planner and solve the problem
//     MyRRT2D planner;
//     amp::Path2D path = planner.plan(problem);

//     // print the solution
//     std::cout << "Found path: ";
//     for (const auto& node : path.nodes) {
//         std::cout << node.position.transpose() << " -> ";
//     }
//     std::cout << std::endl;

//     // visualize the solution
//     amp::Visualizer visualizer;
//     visualizer.visualize(problem, path);

   return 0;
 }
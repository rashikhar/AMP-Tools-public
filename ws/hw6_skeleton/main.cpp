#include "AMPCore.h"
#include "hw/HW6.h"
#include "Q1.h"
#include "Q2.h"
#include "Q3.h"
#include "Q3_Dijkstra.h"

using namespace amp;

class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            return {0, 0};
        }
};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override {
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            return amp::Path2D();
        }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            return amp::Path2D();
        }
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            
            return amp::Path2D();
        }   
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            return GraphSearchResult();
        }
};


void Problem1(){
 // Get the workspace
    Problem2D problem = HW2::getWorkspace2();

//     // Create an instance of your MyPointWFAlgo
//     MyPointWFAlgo wavefrontPlanner;

//     // Set up the start and goal positions for your path planning
//     Eigen::Vector2d qstart(0, 0);
//     Eigen::Vector2d qgoal(10, 10);

//     // Plan the path using your MyPointWFAlgo
//     amp::Path2D generatedPath = wavefrontPlanner.plan(problem, qstart, qgoal);

//     // Calculate the length of the path
//     double pathLength = wavefrontPlanner.calculatePathLength(generatedPath);

//     // Output the path length
//     std::cout << "Path Length: " << pathLength << " units" << std::endl;
// }

// void Problem2(){
//     // Create instances of classes
//     MyCSpaceCtor cspaceConstructor;

//     // Define the robot's initial and goal end-effector positions
//     std::pair<double, double> xstart = {-2.0, 0.0};
//     std::pair<double, double> xgoal = {2.0, 0.0};

//     // Define obstacle information
//     std::vector<Obstacle> obstacles;

//     // Perform inverse kinematics to find robot configurations for start and goal
//     RobotConfiguration startConfig = inverseKinematics(xstart);
//     RobotConfiguration goalConfig = inverseKinematics(xgoal);

//     // Check for collisions along the path
//     if (isCollision(startConfig, obstacles) || isCollision(goalConfig, obstacles)) {
//         std::cout << "Start or goal configuration is in collision. Cannot proceed." << std::endl;
//         return 1;
//     }

//     // Convert robot configurations to grid cells (C-space)
//     std::vector<GridCell> startGridCell = {getCellFromConfiguration(startConfig)};
//     std::vector<GridCell> goalGridCell = {getCellFromConfiguration(goalConfig)};

//     // Perform motion planning using the Wavefront algorithm
//     std::vector<GridCell> path = wavefrontPathPlanning(startGridCell, goalGridCell, cspaceConstructor, obstacles);

//     // Print the path in grid cells
//     std::cout << "Path in grid cells:" << std::endl;
//     for (const GridCell& cell : path) {
//         std::cout << "(" << cell.x << ", " << cell.y << ")" << std::endl;
//     }
// }

// void Problem3(){

//     // Create and populate the graph
//     amp::Graph<double> graph = createGraph();

//     // Define heuristic values
//     std::vector<double> heuristicValues = {4, 3, 3, 3, 3, 3, 2, 3, 1, 1, 1, 1, 2, 0};

//     // Create a problem instance
//     amp::ShortestPathProblem problem{std::make_shared<amp::Graph<double>>(graph), 0, 13};

//     // Create the custom A* algorithm instance
//     MyAStarAlgo customAStar;

//     // Find the optimal path and report results
//     amp::GraphSearchResult result = findOptimalPath(problem, graph);

//     // Output the path, path length, and number of iterations
//     if (!result.optimalPath.empty()) {
//         std::cout << "Optimal path: ";
//         for (amp::Node node : result.optimalPath) {
//             std::cout << "v" << node << " -> ";
//         }
//         std::cout << "v13" << std::endl;
//         std::cout << "Solution cost: " << result.solutionCost << std::endl;
//         std::cout << "Number of iterations: " << result.iterations << std::endl;
//     } else {
//         std::cout << "No path found." << std::endl;
//     }

//     // Visualize the graph
//     visualizeGraph(problem, graph);

// }

// // Implement the functions from Q3.h

// amp::Graph<double> createGraph() {
//     amp::Graph<double> graph;
    
//     // Define nodes and connect them with edges
//     amp::Node v0 = 0;
//     amp::Node v1 = 1;
//     amp::Node v2 = 2;
//     amp::Node v3 = 3;
//     amp::Node v4 = 4;
//     amp::Node v5 = 5;
//     amp::Node v6 = 6;
//     amp::Node v7 = 7;
//     amp::Node v8 = 8;
//     amp::Node v9 = 9;
//     amp::Node v10 = 10;
//     amp::Node v11 = 11;
//     amp::Node v12 = 12;
//     amp::Node v13 = 13;

//     // Connect nodes with edges and specify edge weights
//     graph.connect(v0, v1, 3.0);
//     graph.connect(v0, v2, 1.0);
//     graph.connect(v0, v3, 3.0);
//     graph.connect(v0, v4, 1.0);
//     graph.connect(v0, v5, 3.0);
//     graph.connect(v1, v6, 1.0);
//     graph.connect(v1, v7, 3.0);
//     graph.connect(v2, v1, 0.0);
//     graph.connect(v2, v7, 3.0);
//     graph.connect(v2, v8, 2.0);
//     graph.connect(v2, v9, 1.0);
//     graph.connect(v3, v9, 1.0);
//     graph.connect(v4, v5, 2.0);
//     graph.connect(v4, v9, 1.0);
//     graph.connect(v4, v10, 2.0);
//     graph.connect(v4, v11, 3.0);
//     graph.connect(v5, v11, 1.0);
//     graph.connect(v5, v12, 1.0);
//     graph.connect(v6, v7, 1.0);
//     graph.connect(v7, v13, 1.0);
//     graph.connect(v8, v13, 3.0);
//     graph.connect(v9, v13, 3.0);
//     graph.connect(v10, v13, 3.0);
//     graph.connect(v11, v13, 1.0);

//     return graph;
// }

// amp::GraphSearchResult findOptimalPath(const amp::ShortestPathProblem& problem, amp::Graph<double>& graph) {
//     MyAStarAlgo customAStar;
//     return customAStar.search(problem, amp::SearchHeuristic());
// }

// void visualizeGraph(const amp::ShortestPathProblem& problem, amp::Graph<double>& graph) {
//         amp::GraphTools::makeFigure(problem, graph, [&](amp::Node node) -> Eigen::Vector2d {
//         // Replace with actual coordinates for each node
//         Eigen::Vector2d coordinates;
//         // Fill in coordinates for each node
//         if (node == v0) coordinates << x0, y0;
//         else if (node == v1) coordinates << x1, y1;
//         // Repeat for all nodes v2 to v13
//         return coordinates;
//     });
// }

// void Problem3_Dijkstra(){
//     // Create and populate the graph
//     amp::Graph<double> graph = createGraph();

//     // Define heuristic values (not needed for Dijkstra's algorithm)
//     std::vector<double> heuristicValues = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//     // Create a problem instance
//     amp::ShortestPathProblem problem{std::make_shared<amp::Graph<double>>(graph), 0, 13};

//     // Create the custom Dijkstra algorithm instance
//     MyDijkstraAlgo customDijkstra;

//     // Find the optimal path and report results
//     amp::GraphSearchResult result = findOptimalPath(problem, graph);

//     // Output the path and path length
    // if (!result.optimalPath.empty()) {
    //     std::cout << "Optimal path: ";
    //     for (amp::Node node : result.optimalPath) {
    //         std::cout << "v" << node << " -> ";
    //     }
    //     std::cout << "v13" << std::endl;
    //     std::cout << "Solution cost: " << result.solutionCost << std::endl;

    //     // Output the number of iterations
    //     std::cout << "Number of iterations: " << result.optimalPath.size() - 1 << std::endl;
    // }else {
    //     std::cout << "No path found." << std::endl;
    // }
//     return 0;
// }

int main() {

//     Problem1();
//     Problem2();
//     Problem3();
//     Problem3_Dijkstra();

//     // Create an instance of your Visualizer
//     Visualizer visualizer;
//     // Visualize the path and environment
//     visualizer.makeFigure(problem,generatedPath);
//     Visualizer::makeFigure(problem,generatedPath);
//     // Visualize the path and environment for manipulator
//     visualizeMotionPlan(cspaceConstructor,path);
//     visualizeGraph(problem, graph);
     
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("rashikha.jagula@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;

}
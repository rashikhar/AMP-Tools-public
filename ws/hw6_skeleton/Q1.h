// #pragma once

// #include "AMPCore.h"  
// #include "tools/Environment.h"          
// #include "tools/Visualizer.h"          
// #include <vector>
// #include <cstddef>
// #include <utility>

// namespace std 

// class MyGridCSpace : public amp::GridCSpace2D {
// public:
//     MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
//         : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

//     virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
//         return std::make_pair(0, 0);
//     }
// };

// class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
// public:
//     MyPointWFAlgo() {}

// virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
//     // Extract environment parameters
//     double x0_min = environment.x_min;
//     double x0_max = environment.x_max;
//     double x1_min = environment.y_min;
//     double x1_max = environment.y_max;

//     // Calculate the grid size based on the desired cell size
//     const double cellSize = 0.25;
//     std::size_t x0_cells = static_cast<std::size_t>((x0_max - x0_min) / cellSize);
//     std::size_t x1_cells = static_cast<std::size_t>((x1_max - x1_min) / cellSize);

//     // Create your MyGridCSpace with the calculated parameters
//     return std::make_unique<MyGridCSpace>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);
// }


//     virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
//         // Implement the Wave-Front path planning algorithm here
        
//         // Extract your specific initial and goal positions
//         Eigen::Vector2d qstart(0, 0); 
//         Eigen::Vector2d qgoal(10, 10); 

//         // Calculate the grid cell corresponding to the initial and goal positions
//         int startCol = grid_cspace.cellCol(qstart.x());
//         int startRow = grid_cspace.cellRow(qstart.y());
//         int goalCol = grid_cspace.cellCol(qgoal.x());
//         int goalRow = grid_cspace.cellRow(qgoal.y());

//         // Create a 2D grid to store the wavefront values
//         std::vector<std::vector<int>> wavefront(grid_cspace.numCellsX(), std::vector<int>(grid_cspace.numCellsY(), -1));

//         // Initialize the starting cell with a wavefront value of 0
//         wavefront[startCol][startRow] = 0;

//         // Perform wavefront propagation
//         bool goalReached = false;
//         int wavefrontValue = 0;

//         while (!goalReached) {
//             goalReached = true;

//             for (int row = 0; row < grid_cspace.numCellsY(); ++row) {
//                 for (int col = 0; col < grid_cspace.numCellsX(); ++col) {
//                     if (wavefront[col][row] == wavefrontValue) {
//                         // Expand the wavefront to neighboring cells

//                         // Check the neighboring cells (up, down, left, right)
//                         int neighborsX[] = {0, 0, -1, 1};
//                         int neighborsY[] = {-1, 1, 0, 0};

//                         for (int i = 0; i < 4; ++i) {
//                             int neighborCol = col + neighborsX[i];
//                             int neighborRow = row + neighborsY[i];

//                             // Check if the neighbor is within the grid
//                             if (neighborCol >= 0 && neighborCol < grid_cspace.numCellsX() &&
//                                 neighborRow >= 0 && neighborRow < grid_cspace.numCellsY()) {

//                                 // Check if the neighbor cell is unvisited (wavefront value is -1)
//                                 if (wavefront[neighborCol][neighborRow] == -1) {
//                                     // Update the wavefront value of the neighbor cell
//                                     wavefront[neighborCol][neighborRow] = wavefrontValue + 1;

//                                     // Mark goalReached as false if the goal is found in the neighbor cell
//                                     if (neighborCol == goalCol && neighborRow == goalRow) {
//                                         goalReached = false;
//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }

//             wavefrontValue++;
//         }

//         // Generate the path from the goal back to the start
//         std::vector<Eigen::Vector2d> pathPoints;
//         int currentCol = goalCol;
//         int currentRow = goalRow;

//         while (currentCol != startCol || currentRow != startRow) {
//             pathPoints.emplace_back(grid_cspace.cellCenter(currentCol, currentRow));
//             // Backtrack from goal to start
//         }

//         // Reverse the path
//         std::reverse(pathPoints.begin(), pathPoints.end());

//         // Convert the path to amp::Path2D
//         amp::Path2D generatedPath;
//         for (const auto& point : pathPoints) {
//             generatedPath.append(point);
//         }

//         return generatedPath;
//     }
// };

//     double calculatePathLength(const amp::Path2D& path) {
//         double length = 0.0;
//         Eigen::Vector2d prevPoint = path.front();

//         for (const Eigen::Vector2d& point : path) {
//             // Calculate the Euclidean distance between the previous point and the current point
//             double dx = point.x() - prevPoint.x();
//             double dy = point.y() - prevPoint.y();
//             double segmentLength = std::sqrt(dx * dx + dy * dy);

//             length += segmentLength;
//             prevPoint = point;
//         }

//         return length;
//     };


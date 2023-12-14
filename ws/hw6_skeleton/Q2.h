// #include <iostream>
// #include <cmath>


// // Robot Configuration (Joint Angles)
// struct RobotConfiguration {
//     double theta1;
//     double theta2;
// };
// double degreesToRadians(double degrees) {
//     return degrees * M_PI / 180.0;
// }

// // Obstacle Representation (Polygon Vertices)
// struct Obstacle {
//     std::vector<std::pair<double, double>> vertices;
// };

// // Forward Kinematics to Compute End-Effector Position
// std::pair<double, double> forwardKinematics(const RobotConfiguration& config) {
//     double l1 = 1.0;  // Length of link 1
//     double l2 = 1.0;  // Length of link 2

//     double x = l1 * cos(config.theta1) + l2 * cos(config.theta1 + config.theta2);
//     double y = l1 * sin(config.theta1) + l2 * sin(config.theta1 + config.theta2);

//     return {theta1, theta2};
//     return {x, y};
// }

// // Inverse Kinematics to Find Joint Angles from End-Effector Position
// RobotConfiguration inverseKinematics(const std::pair<double, double>& endEffectorPos) {
//     RobotConfiguration config;
//     // Implement your inverse kinematics calculations here

//     double x = endEffectorPos.first;
//     double y = endEffectorPos.second;
//     config.theta1 = atan2(y, x);  // Angle of link 1
//     config.theta2 = 0.0;  // Placeholder value for the second joint

//     return config;
// }

// // Check for collision between the robot and obstacles
// bool isCollision(const RobotConfiguration& config, const std::vector<Obstacle>& obstacles) {
//     // Extract the positions of robot vertices
//     double l1 = 1.0;  // Length of link 1
//     double l2 = 1.0;  // Length of link 2
//     double x1 = l1 * cos(config.theta1);
//     double y1 = l1 * sin(config.theta1);
//     double x2 = x1 + l2 * cos(config.theta1 + config.theta2);
//     double y2 = y1 + l2 * sin(config.theta1 + config.theta2);

//     // Check for collision with each obstacle
//     for (const Obstacle& obstacle : obstacles) {
//         if (isPointInPolygon({x1, y1}, obstacle) || isPointInPolygon({x2, y2}, obstacle)) {
//             return true;  // Collision detected
//         }
//     }

//     return false;  // No collision
// }

// // Check if a point is inside a polygon (used for collision detection)
// bool isPointInPolygon(const std::pair<double, double>& point, const Obstacle& obstacle) {
//     int numVertices = obstacle.vertices.size();
//     int j = numVertices - 1;
//     bool inside = false;

//     for (int i = 0; i < numVertices; j = i++) {
//         double xi = obstacle.vertices[i].first;
//         double yi = obstacle.vertices[i].second;
//         double xj = obstacle.vertices[j].first;
//         double yj = obstacle.vertices[j].second;

//         bool intersect = ((yi > point.second) != (yj > point.second)) &&
//                         (point.first < (xj - xi) * (point.second - yi) / (yj - yi) + xi);
//         if (intersect) inside = !inside;
//     }

//     return inside;
// }

// // Implement the Wavefront algorithm for path planning in C-space
// std::vector<GridCell> wavefrontPathPlanning(const std::vector<GridCell>& start,
//                                            const std::vector<GridCell>& goal,
//                                            MyCSpaceCtor& cspaceConstructor,
//                                            const std::vector<Obstacle>& obstacles) {
//     // Create instances of your custom classes
//     MyManipWFAlgo wavefrontPlanner;

//     // Perform motion planning using the Wavefront algorithm
//     std::vector<GridCell> path;

//     std::queue<GridCell> frontier;
//     std::vector<std::vector<int>> wavefrontGrid(cspaceConstructor.getGridSizeX(), std::vector<int>(cspaceConstructor.getGridSizeY(), -1));

//     for (const GridCell& goalCell : goal) {
//         frontier.push(goalCell);
//         wavefrontGrid[goalCell.x][goalCell.y] = 0;
//     }

//     while (!frontier.empty()) {
//         GridCell current = frontier.front();
//         frontier.pop();

//         for (const GridCell& neighbor : getNeighbors(current)) {
//             if (neighbor.x >= 0 && neighbor.x < cspaceConstructor.getGridSizeX() &&
//                 neighbor.y >= 0 && neighbor.y < cspaceConstructor.getGridSizeY() &&
//                 wavefrontGrid[neighbor.x][neighbor.y] == -1 &&
//                 !isCollision(getConfigurationFromCell(neighbor), obstacles)) {
//                 wavefrontGrid[neighbor.x][neighbor.y] = wavefrontGrid[current.x][current.y] + 1;
//                 frontier.push(neighbor);
//             }
//         }
//     }

//     // Reconstruct the path using the Wavefront grid
//     GridCell current = start[0];  // Assuming a single starting cell
//     path.push_back(current);
//     int currentWavefrontValue = wavefrontGrid[current.x][current.y];

//     while (current != goal[0]) {  // Assuming a single goal cell
//         for (const GridCell& neighbor : getNeighbors(current)) {
//             if (neighbor.x >= 0 && neighbor.x < cspaceConstructor.getGridSizeX() &&
//                 neighbor.y >= 0 && neighbor.y < cspaceConstructor.getGridSizeY() &&
//                 wavefrontGrid[neighbor.x][neighbor.y] == currentWavefrontValue - 1) {
//                 current = neighbor;
//                 path.push_back(current);
//                 currentWavefrontValue--;
//                 break;
//             }
//         }
//     }

//     return path;
// }

// // Reconstruct the path using the Wavefront grid
//     GridCell current = start[0];  // Assuming a single starting cell
//     path.push_back(current);
//     int currentWavefrontValue = wavefrontGrid[current.x][current.y];

//     while (current != goal[0]) {  // Assuming a single goal cell
//         for (const GridCell& neighbor : getNeighbors(current)) {
//             if (neighbor.x >= 0 && neighbor.x < cspaceConstructor.getGridSizeX() &&
//                 neighbor.y >= 0 && neighbor.y < cspaceConstructor.getGridSizeY() &&
//                 wavefrontGrid[neighbor.x][neighbor.y] == currentWavefrontValue - 1) {
//                 current = neighbor;
//                 path.push_back(current);
//                 currentWavefrontValue--;
//                 break;
//             }
//         }
//     }

//     return path;



// int main() {
//     // Define the robot's initial and goal end-effector positions
//     std::pair<double, double> xstart = {-2.0, 0.0};
//     std::pair<double, double> xgoal = {2.0, 0.0};

//     // Define obstacles
//     Obstacle obstacle;
//     obstacle.vertices = {{0.5, 0.5}, {0.5, 1.0}, {1.0, 1.0}, {1.0, 0.5}};

//     std::vector<Obstacle> obstacles = {obstacle};

//     // Perform motion planning using inverse kinematics
//     RobotConfiguration startConfig = inverseKinematics(xstart);
//     RobotConfiguration goalConfig = inverseKinematics(xgoal);

//     // Check for collisions
//     if (isCollision(startConfig, obstacles) || isCollision(goalConfig, obstacles)) {
//         std::cout << "Start or goal configuration is in collision. Cannot proceed." << std::endl;
//         return 1;
//     }

//     return 0;
// }

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <cmath>

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace Eigen;
using namespace amp;

// Problem1a
// Define a function to calculate the C-space vertices using Minkowski difference
// std::vector<Eigen::Vector2d> calculateCSpaceVertices(const std::vector<Eigen::Vector2d>& obstacleVertices, const std::vector<Eigen::Vector2d>& robotVertices) {
//     std::vector<Eigen::Vector2d> cspaceVertices;

//         for (const Eigen::Vector2d& obstacleVertex : obstacleVertices) {
//         for (const Eigen::Vector2d& robotVertex : robotVertices) {
//             Eigen::Vector2d cspaceVertex = obstacleVertex - robotVertex;
//             cspaceVertices.push_back(cspaceVertex);
//         }
//     }

//     return cspaceVertices;
// }



// Problem 1b
// Define a function to calculate the C-space vertices using Minkowski difference
std::vector<Eigen::Vector2d> calculateCSpaceVertices(const std::vector<Eigen::Vector2d>& obstacleVertices, const std::vector<Eigen::Vector2d>& robotVertices) {
    std::vector<Eigen::Vector2d> cspaceVertices;
    std::vector<Eigen::Vector2d> cspaceVertex;

        for (const Eigen::Vector2d& obstacleVertex : obstacleVertices) {
        for (const Eigen::Vector2d& robotVertex : robotVertices){
        Eigen::Vector2d cspaceVertex = obstacleVertex - robotVertex;
        cspaceVertices.push_back(cspaceVertex);}
    }

    return cspaceVertices;
}


// Problem 2a
// 3 Link manipulator
// class Planar3LinkManipulator : public LinkManipulator2D {
// public:
//     Planar3LinkManipulator()
//         : LinkManipulator2D({0.5, 1.0, 0.5}) {} // Link lengths: a1 = 0.5, a2 = 1.0, a3 = 0.5

//     Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override {
//         if (joint_index >= nLinks()) {
//             std::cerr << "Invalid joint index: " << joint_index << std::endl;
//             return Eigen::Vector2d(0.0, 0.0);
//         }

//         Eigen::Vector2d joint_location(0.0, 0.0);
//         for (uint32_t i = 0; i <= joint_index; ++i) {
//             joint_location += Eigen::Vector2d(state[i], 0.0);
//         }

//         return joint_location;
//     }
// };


//Problem2b
//3 link manipulator
// Define a struct to represent the manipulator configuration
// struct ManipulatorConfiguration {
//     double theta1;
//     double theta2;
//     double theta3;
// };

// // Function to calculate joint angles using inverse kinematics
// vector<ManipulatorConfiguration> calculateIK(double xD, double yD, double L1, double L2, double L3) {
//     vector<ManipulatorConfiguration> configurations;

//     // Calculate θ1 using inverse tangent
//     vector<double> theta1_possible = {atan2(yD, xD)};

//     for (double theta1 : theta1_possible) {
//         // Calculate the distance from the base to the end effector
//         double r = sqrt(xD * xD + yD * yD);

//         // Calculate the cosine of θ3 using the law of cosines
//         double cos_theta3 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);

//         // Check if cos_theta3 is within [-1, 1] (valid range for acos)
//         if (abs(cos_theta3) <= 1) {
//             // Calculate θ3 using the inverse cosine function
//             double theta3_possible = acos(cos_theta3);

//             // Calculate θ2 using the law of sines
//             double sin_theta3 = sqrt(1 - cos_theta3 * cos_theta3);
//             double sin_theta2 = (L2 / r) * sin_theta3;
//             double cos_theta2 = (L1 + L2 * cos_theta3) / r;

//             // Check if sin_theta2 and cos_theta2 are within [-1, 1] (valid range for asin and acos)
//             if (abs(sin_theta2) <= 1 && abs(cos_theta2) <= 1) {
//                 // Calculate θ2 using the inverse sine function
//                 double theta2_possible = atan2(sin_theta2, cos_theta2);

//                 // Create a configuration
//                 ManipulatorConfiguration config;
//                 config.theta1 = theta1;
//                 config.theta2 = theta2_possible;
//                 config.theta3 = theta3_possible;

//                 // Append the configuration to the list
//                 configurations.push_back(config);
//             }
//         }
//     }

//     return configurations;
// }


// Problem 3
// 2link manipulator in 2dspace
// const double collisionThreshold = 0.1; // Adjust this threshold as needed

// // Function to check for collision using Minkowski difference
// bool isCollision(const amp::LinkManipulator2D& manipulator, const amp::Obstacle2D& obstacle) {
//     for (const auto& link : manipulator.links) {
//         for (const auto& vertex : obstacle.vertices) {
//             // Calculate Minkowski difference
//             Eigen::Vector2d minkowskiDiff = link.end() - link.start() - vertex;

//             // Check for collision (assuming a certain threshold)
//             if (minkowskiDiff.norm() < collisionThreshold) {
//                 return true; // Collision detected
//             }
//         }
//     }
//     return false; // No collision detected
// }


//  void Problem1a(){
//     // Define the vertices of the obstacle triangle in workspace W
//     std::vector<Eigen::Vector2d> obstacleVertices = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(0.0, 2.0)};

//     // Define the local reference point of the robot at its lower-left vertex
//      std::vector<Eigen::Vector2d> robotVertices = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(0.0, 2.0)};

//     // Calculate the vertices of the C-space obstacle using Minkowski difference
//     std::vector<Eigen::Vector2d> cspaceVertices = calculateCSpaceVertices(obstacleVertices, robotVertices);

//     // Print the C-space vertices
//     std::cout << "Vertices of the C-space obstacle:" << std::endl;
//     for (const Eigen::Vector2d& vertex : cspaceVertices) {
//         std::cout << "(" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
//     }
//     // Print the C-space vertices
//     std::cout << "Vertices of the C-space obstacle:" << std::endl;
//     for (const Eigen::Vector2d& vertex : cspaceVertices) {
//         std::cout << "(" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
//     }

//     // Create an Environment2D to represent the obstacle
//     std::vector<Polygon> polygons;
//     Polygon polygon(obstacleVertices);
//     polygons.push_back(polygon);


//     // Visualize the obstacle and C-space obstacle
//     Visualizer::makeFigure(polygons);  // Display the obstacle in red
//    //Visualizer::makeFigure(cspace.getPolygonVertices(), true);  // Display the C-space obstacle in green

//     // Show the figures (blocking until closed)
//     Visualizer::showFigures();

// }



void Problem1b(){
// // Define the vertices of the obstacle triangle in workspace W
    std::vector<Eigen::Vector2d> obstacleVertices = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(0.0, 2.0)};

    /// Define the local reference point of the robot at its lower-left vertex
     std::vector<Eigen::Vector2d> robotVertices = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(0.0, 2.0)};

    /// Discretize the additional parameter (angle of rotation)
    const int numAngles = 12;
    const double angleIncrement = 2.0 * M_PI / numAngles;

    /// Create a vector to store the C-space obstacles for different angles
    std::vector<Environment2D> cspaceObstacles;
    std::vector<Polygon> polygons;

    /// Calculate the C-space obstacles for each angle
    std::vector<Eigen::Vector2d> rotatedObstacleVertices;

    for (int i = 0; i < numAngles; ++i) {
        double angle = i * angleIncrement;

        /// Calculate the rotated obstacle vertices
        std::vector<Eigen::Vector2d> rotatedObstacleVertices;
        for (const Eigen::Vector2d& vertex : obstacleVertices) {
            Eigen::Vector2d rotatedVertex;
            rotatedVertex.x() = vertex.x() * cos(angle) - vertex.y() * sin(angle);
            rotatedVertex.y() = vertex.x() * sin(angle) + vertex.y() * cos(angle);
            rotatedObstacleVertices.push_back(rotatedVertex);
        }

        /// Calculate the vertices of the C-space obstacle using Minkowski difference
        std::vector<Eigen::Vector2d> cspaceVertices = calculateCSpaceVertices(rotatedObstacleVertices, robotVertices);
        polygons.push_back(Polygon(rotatedObstacleVertices));

        /// Create an Environment2D to represent the obstacle
        Environment2D env;
        env.obstacles.push_back(Polygon(rotatedObstacleVertices));
        std::vector<Polygon> polygons;
        Polygon polygon(rotatedObstacleVertices);
        polygons.push_back(polygon);

        /// Create a ConfigurationSpace2D to calculate the C-space
        // ConfigurationSpace2D cspace(rotatedObstacleVertices);

        /// Calculate the C-space using Minkowski difference
        // cspace.calculate();

        /// Store the C-space obstacle in the vector
        cspaceObstacles.push_back(env);
        }

    /// Visualize the sequence of C-space obstacles (requires AMP Visualizer)
    Visualizer::makeFigure(polygons); // Display the C-space obstacles in green

    /// Show the figures (blocking until closed)
    Visualizer::showFigures();
}


// void Problem2a() {
// class Planar3LinkManipulator {
// public:
//     Planar3LinkManipulator(double l1, double l2, double l3) : l1_(l1), l2_(l2), l3_(l3) {}

//     void calculateConfiguration(const std::vector<double>& joint_angles) {
//         if (joint_angles.size() != 3) {
//             std::cerr << "Invalid number of joint angles. Expected 3." << std::endl;
//             return;
//         }

//         double theta1 = joint_angles[0];
//         double theta2 = joint_angles[1];
//         double theta3 = joint_angles[2];

//         double x = l1_ * cos(theta1) + l2_ * cos(theta1 + theta2) + l3_ * cos(theta1 + theta2 + theta3);
//         double y = l1_ * sin(theta1) + l2_ * sin(theta1 + theta2) + l3_ * sin(theta1 + theta2 + theta3);

//         std::cout << "End-Effector Position: (" << x << ", " << y << ")" << std::endl;

//         // Visualize the robot configuration
//         // Visualizer::makeFigure(Vector2d(0.0, 0.0), Vector2d(x, y));
//         // Visualizer::showFigures();
    
//     }

// private:
//     double l1_;
//     double l2_;
//     double l3_;
// };
// };

// void Problem2b(){
//     double l1, l2, l3;
//     std::cout << "Enter link lengths (l1 l2 l3): ";
//     std::cin >> l1 >> l2 >> l3;

//     Planar3LinkManipulator manipulator(l1, l2, l3);

//     std::vector<double> joint_angles(3);
//     std::cout << "Enter joint angles (in radians, θ1 θ2 θ3): ";
//     std::cin >> joint_angles[0] >> joint_angles[1] >> joint_angles[2];

//     manipulator.calculateConfiguration(joint_angles);
// }

// Void Problem3(){
// // User-specified parameters
//     double link1_length = 1.0;
//     double link2_length = 1.0;
//     int num_obstacles = 0; // Number of obstacles (user input)

//     std::cout << "Enter the length of the first link: ";
//     std::cin >> link1_length;

//     std::cout << "Enter the length of the second link: ";
//     std::cin >> link2_length;

//     std::cout << "Enter the number of obstacles: ";
//     std::cin >> num_obstacles;

//     // Create a workspace environment
//     amp::Environment2D env;
//     env.x_max = 10.0;
//     env.y_max = 10.0;

//     // Create a 2-link manipulator
//     std::vector<double> link_lengths = {link1_length, link2_length};
//     amp::LinkManipulator2D manipulator(link_lengths);

//     // Add obstacles to the environment
//     for (int obstacle_index = 0; obstacle_index < num_obstacles; ++obstacle_index) {
//         std::cout << "Enter the number of vertices for obstacle " << obstacle_index + 1 << ": ";
//         int num_vertices = 0;
//         std::cin >> num_vertices;

//         std::vector<Eigen::Vector2d> obstacle_vertices;
//         std::cout << "Enter the vertices (x, y) of the obstacle in counterclockwise order:" << std::endl;
//         for (int vertex_index = 0; vertex_index < num_vertices; ++vertex_index) {
//             double x, y;
//             std::cout << "Vertex " << vertex_index + 1 << ": ";
//             std::cin >> x >> y;
//             obstacle_vertices.push_back(Eigen::Vector2d(x, y));
//         }

//         // Add the obstacle to the environment
//         amp::Obstacle2D obstacle(obstacle_vertices);
//         env.obstacles.push_back(obstacle);
//     }

//     // Create a grid C-space
//     std::size_t grid_resolution = 100; // Fine grid resolution
//     amp::GridCSpace2D cspace(grid_resolution, grid_resolution, 0.0, env.x_max, 0.0, env.y_max);

//     // Populate the grid C-space by checking collisions
//     for (std::size_t i = 0; i < grid_resolution; ++i) {
//         for (std::size_t j = 0; j < grid_resolution; ++j) {
//             // Calculate the configuration (x, y) in the grid C-space
//             double x = i * (env.x_max / static_cast<double>(grid_resolution));
//             double y = j * (env.y_max / static_cast<double>(grid_resolution));

//             // Set the configuration of the manipulator
//             manipulator.setConfiguration(x, y);

//             // Check for collision with each obstacle
//             bool collision = false;
//             for (const auto& obstacle : env.obstacles) {
//                 if (isCollision(manipulator, obstacle)) {
//                     collision = true;
//                     break; // No need to check with other obstacles if collision detected
//                 }
//             }

//             // Set the collision status in the grid C-space
//             cspace.setCollisionStatus(i, j, collision);
//         }
//     }

//     // Visualize the workspace with obstacles
//     amp::Visualizer::makeFigure(env);

//     // Visualize the C-space with obstacles (positive values and whole space)
//     amp::Visualizer::createAxes(env);
//     amp::Visualizer::createAxes(cspace); // Use the custom axis bounds for C-space visualization
//     amp::Visualizer::makeFigure(cspace);

//     // Show the visualization
//     amp::Visualizer::showFigures();

// }

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // Problem1a();
    Problem1b();
    // Problem2a();
    //Problem2b();
    //Problem3();
    Visualizer::showFigures();

    // Grade method
    // amp::HW4::grade<LinkManipulator2D>(constructor, "rashikha.jagula@colorado.edu", argc, argv);

   return 0;

}
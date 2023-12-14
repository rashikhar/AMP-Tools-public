#include <iostream>
#include <vector>
#include <cmath>

struct Point {
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

const Point qstart(0, 0);
const Point qgoal(10, 0);

// Define the obstacle centers and radii
const std::vector<Point> obstacles = {
    Point(4, 1),
    Point(7, -1)
};
const double obstacle_radius = 0.5;  // Half the side length of the square obstacle

// Define parameters for the gradient descent
const double learning_rate = 0.1;  // Step size for gradient descent
const double goal_radius = 0.25;   // Termination condition radius at the goal

// Function to calculate the attractive potential
double CalculateAttractivePotential(const Point& q, double zeta, double dgoal) {
    double distance = std::hypot(q.x - qgoal.x, q.y - qgoal.y);
    if (distance <= dgoal) {
        return 0.5 * zeta * distance * distance;
    } else {
        return dgoal * zeta * distance - 0.5 * zeta * dgoal * dgoal;
    }
}

// Function to calculate the repulsive potential
double CalculateRepulsivePotential(const Point& q, double eta, double Qi, const std::vector<Point>& obstacles) {
    double potential = 0.0;

    for (const Point& obstacle : obstacles) {
        double distance_to_obstacle = std::hypot(q.x - obstacle.x, q.y - obstacle.y);
        
        if (distance_to_obstacle < Qi) {
            double repulsive_force = 1.0 / distance_to_obstacle - 1.0 / Qi;
            potential += 0.5 * eta * repulsive_force * repulsive_force;
        }
    }

    return potential;
}

// Function to perform gradient descent
Point GradientDescent(const Point& current_point, double zeta, double eta, double dgoal, double Qi) {
    // Calculate the attractive and repulsive potentials
    double Uatt = CalculateAttractivePotential(current_point, zeta, dgoal);
    double Urep = CalculateRepulsivePotential(current_point, eta, Qi, obstacles);
    
    // Combine the potentials
    double U = Uatt + Urep;

    // Calculate the gradient
    double gradient_x = -(zeta * (current_point.x - qgoal.x) + eta * (current_point.x - qgoal.x) / Qi) + U;
    double gradient_y = -(zeta * (current_point.y - qgoal.y) + eta * (current_point.y - qgoal.y) / Qi) + U;
    
    // Update the position
    double new_x = current_point.x - learning_rate * gradient_x;
    double new_y = current_point.y - learning_rate * gradient_y;
    
    return Point(new_x, new_y);
}

// Function to calculate the length of a path
double CalculatePathLength(const std::vector<Point>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        length += std::hypot(dx, dy);
    }
    return length;
}

int main() {
    Point current_point = qstart;
    double zeta = 1.0;  // Attractive potential strength
    double eta = 1.0;   // Repulsive potential strength
    double dgoal = 1.0;  // Desired distance from the goal
    double Qi = 1.0;     // Influence range of obstacles

    std::vector<Point> path;
    path.push_back(current_point);

    while (std::hypot(current_point.x - qgoal.x, current_point.y - qgoal.y) > goal_radius) {
        current_point = GradientDescent(current_point, zeta, eta, dgoal, Qi);
        path.push_back(current_point);
    }

    std::cout << "Reached the goal: (" << current_point.x << ", " << current_point.y << ")\n";

    double path_length = CalculatePathLength(path);
    std::cout << "Length of the generated path: " << path_length << std::endl;
    
    return 0;
}

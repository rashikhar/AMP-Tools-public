// #pragma once

// #include "Graph.h"

// // Define a custom Dijkstra algorithm class
// class MyDijkstraAlgo {
// public:
//     amp::GraphSearchResult search(const amp::ShortestPathProblem& problem);
// };

// // Function to create and populate the graph
// amp::Graph<double> createGraph();

// // Function to find the optimal path and report results
// amp::GraphSearchResult findOptimalPath(const amp::ShortestPathProblem& problem, amp::Graph<double>& graph);

// amp::GraphSearchResult MyDijkstraAlgo::search(const amp::ShortestPathProblem& problem) {
//     amp::GraphSearchResult result;

//     const auto& graph = *problem.graph;  // Access the graph from the problem

//     int start = problem.init_node;
//     int goal = problem.goal_node;

//     // Priority queue for nodes to be explored
//     std::priority_queue<amp::Node, std::vector<amp::Node>, std::function<bool(amp::Node, amp::Node)>> open(
//         [](amp::Node a, amp::Node b) { return a.f > b.f; });

//     // Cost from start along the best-known path
//     std::vector<double> g(graph.size(), std::numeric_limits<double>::max());
//     g[start] = 0;

//     open.push({start, 0, 0});

//     // To reconstruct the path
//     std::vector<int> cameFrom(graph.size(), -1);

//     while (!open.empty()) {
//         amp::Node current = open.top();
//         open.pop();

//         if (current.id == goal) {
//             // Reconstruct the path
//             std::vector<int> path;
//             int node = goal;
//             while (node != start) {
//                 path.push_back(node);
//                 node = cameFrom[node];
//             }
//             path.push_back(start);
//             std::reverse(path.begin(), path.end());

//             result.optimalPath = path;
//             result.solutionCost = g[goal];
//             return result;
//         }

//         for (auto child : graph.children(current.id)) {
//             double tentativeG = g[current.id] + graph.outgoingEdges(current.id)[child] /* edge weight */;
//             if (tentativeG < g[child]) {
//                 g[child] = tentativeG;
//                 cameFrom[child] = current.id;
//                 open.push({child, g[child], 0});
//             }
//         }
//     }

//     // No path found
//     result.optimalPath = {};
//     result.solutionCost = std::numeric_limits<double>::max();
//     return result;
// }

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
//     MyDijkstraAlgo customDijkstra;
//     return customDijkstra.search(problem);
// }
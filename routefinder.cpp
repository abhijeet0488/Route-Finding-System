#include <iostream> // For input/output operations
#include <vector>   // For dynamic arrays (adjacency list)
#include <queue>    // For priority queue (used in Dijkstra's)
#include <map>      // For mapping city names to integer IDs
#include <limits>   // For numeric_limits (representing infinity)
#include <string>   // For string manipulation
#include <algorithm> // For std::reverse and std::find_if

// Define infinity for distance calculations
const int INF = std::numeric_limits<int>::max();

// Structure to represent an edge in the graph
struct Edge {
    int to_node; // The destination node (city ID)
    int weight;  // The weight of the edge (distance/cost)

    // Constructor
    Edge(int to, int w) : to_node(to), weight(w) {}
};

// Custom comparator for the priority queue
// It orders pairs based on their first element (distance) in ascending order
// This creates a min-priority queue for Dijkstra's.
struct CompareDist {
    bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return a.first > b.first; // Min-heap based on distance
    }
};

class Graph {
private:
    std::vector<std::vector<Edge>> adj_list; // Adjacency list to store the graph
    std::map<std::string, int> city_to_id;   // Map city names to unique integer IDs
    std::vector<std::string> id_to_city;     // Map integer IDs back to city names
    int num_nodes;                           // Total number of cities/nodes

    // Private helper method to add a directed edge
    void add_road_directed(const std::string& from_city, const std::string& to_city, int distance) {
        int from_id = add_city(from_city); // Ensure 'from_city' exists and get its ID
        int to_id = add_city(to_city);     // Ensure 'to_city' exists and get its ID
        adj_list[from_id].emplace_back(to_id, distance); // Add edge to adjacency list
    }

public:
    // Constructor to initialize the graph
    Graph() : num_nodes(0) {}

    // Method to add a city to the graph
    int add_city(const std::string& city_name) {
        if (city_to_id.find(city_name) == city_to_id.end()) {
            // If city does not exist, assign a new ID
            city_to_id[city_name] = num_nodes;
            id_to_city.push_back(city_name);
            adj_list.resize(num_nodes + 1); // Resize adjacency list to accommodate new node
            num_nodes++;
        }
        return city_to_id[city_name]; // Return the ID of the city
    }

    // Public method to add a road, with an option for bidirectional
    void add_road(const std::string& from_city, const std::string& to_city, int distance, bool bidirectional = false) {
        add_road_directed(from_city, to_city, distance);
        if (bidirectional) {
            add_road_directed(to_city, from_city, distance); // Add the reverse edge for bidirectional
        }
    }

    // Method to update the weight of an existing road
    bool update_road_weight(const std::string& from_city, const std::string& to_city, int new_distance) {
        auto from_it = city_to_id.find(from_city);
        auto to_it = city_to_id.find(to_city);

        if (from_it == city_to_id.end() || to_it == city_to_id.end()) {
            std::cerr << "Error: One or both cities for weight update not found.\n";
            return false;
        }

        int from_id = from_it->second;
        int to_id = to_it->second;

        // Find the edge and update its weight
        bool updated = false;
        for (Edge& edge : adj_list[from_id]) {
            if (edge.to_node == to_id) {
                edge.weight = new_distance;
                updated = true;
                break;
            }
        }
        if (!updated) {
            std::cerr << "Warning: Road from " << from_city << " to " << to_city << " not found for update. Adding it as a new road.\n";
            add_road_directed(from_city, to_city, new_distance); // Add if not found
            updated = true; // Consider it updated as it's now present with new weight
        }
        return updated;
    }


    // Dijkstra's Algorithm to find the shortest path from a start city to all other cities
    // Returns a pair: {distances vector, predecessors vector}
    std::pair<std::vector<int>, std::vector<int>> dijkstra(const std::string& start_city) {
        int start_node;
        // Check if the start city exists
        if (city_to_id.find(start_city) == city_to_id.end()) {
            std::cerr << "Error: Start city '" << start_city << "' not found.\n";
            return {{}, {}}; // Return empty vectors on error
        }
        start_node = city_to_id[start_city];

        std::vector<int> dist(num_nodes, INF);     // Stores shortest distance from start_node to all other nodes
        std::vector<int> prev(num_nodes, -1);      // Stores predecessor node in the shortest path
        // Priority queue: stores pairs of (distance, node_id)
        // Orders by smallest distance first.
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, CompareDist> pq;

        dist[start_node] = 0; // Distance to start node is 0
        pq.push({0, start_node}); // Push start node to priority queue

        std::cout << "\n--- Dijkstra's Algorithm Trace ---\n";
        std::cout << "Starting from: " << start_city << "\n";

        while (!pq.empty()) {
            int d = pq.top().first;  // Current shortest distance to this node
            int u = pq.top().second; // Current node (city ID)
            pq.pop();

            // Text-based visualization: Node being processed
            std::cout << "Processing node: " << id_to_city[u] << " (Current dist: " << d << ")\n";

            // If we found a shorter path to 'u' already, skip this entry
            if (d > dist[u]) {
                std::cout << "  (Skipping, already found a shorter path to " << id_to_city[u] << ")\n";
                continue;
            }

            // Iterate through all neighbors of 'u'
            for (const Edge& edge : adj_list[u]) {
                int v = edge.to_node;
                int weight = edge.weight;

                // If a shorter path to 'v' is found through 'u'
                if (dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight; // Update shortest distance to 'v'
                    prev[v] = u;                 // Set 'u' as predecessor of 'v'
                    pq.push({dist[v], v});       // Push 'v' with its new distance to PQ
                    // Text-based visualization: Distance update
                    std::cout << "  Updating path to " << id_to_city[v] << " via " << id_to_city[u] 
                              << ". New distance: " << dist[v] << "\n";
                } else {
                    // Text-based visualization: Edge relaxation not improving path
                    if (dist[u] != INF) { // Only if u is reachable
                        std::cout << "  Considering edge " << id_to_city[u] << " -> " << id_to_city[v] 
                                  << " (Weight: " << weight << "). No improvement.\n";
                    }
                }
            }
        }
        std::cout << "--- Dijkstra's Algorithm Trace End ---\n";
        return {dist, prev}; // Return calculated distances and predecessors
    }

    // Helper to reconstruct the path from predecessors
    std::vector<std::string> reconstruct_path(int start_node, int end_node, const std::vector<int>& prev) {
        std::vector<std::string> path;
        // Check if path exists (end_node is reachable and has a predecessor, unless it's the start node itself)
        if (prev[end_node] == -1 && start_node != end_node) {
            return {}; // No path found
        }

        int current = end_node;
        while (current != -1) {
            // Ensure current node ID is valid before accessing id_to_city
            if (current < 0 || current >= id_to_city.size()) {
                // This indicates an error in path reconstruction or prev array
                std::cerr << "Error: Invalid node ID " << current << " during path reconstruction.\n";
                return {}; 
            }
            path.push_back(id_to_city[current]); // Add city name to path
            if (current == start_node) break; // Stop if we reached the start node
            current = prev[current]; // Move to predecessor
        }
        std::reverse(path.begin(), path.end()); // Reverse to get path from start to end
        return path;
    }

    // Main function to find and display the shortest path between two cities
    void find_shortest_path(const std::string& start_city_name, const std::string& end_city_name) {
        // Check if end city exists
        if (city_to_id.find(end_city_name) == city_to_id.end()) {
            std::cerr << "Error: End city '" << end_city_name << "' not found.\n";
            return;
        }

        auto result = dijkstra(start_city_name); // Run Dijkstra's from start city
        std::vector<int> dists = result.first;
        std::vector<int> prevs = result.second;

        if (dists.empty() || city_to_id.find(start_city_name) == city_to_id.end()) { 
            // Error occurred in dijkstra (e.g., start city not found) or start city invalid
            return;
        }

        int start_node_id = city_to_id[start_city_name];
        int end_node_id = city_to_id[end_city_name];

        std::vector<std::string> path = reconstruct_path(start_node_id, end_node_id, prevs);

        if (dists[end_node_id] == INF) {
            std::cout << "No path found from " << start_city_name << " to " << end_city_name << "\n";
        } else {
            std::cout << "\nShortest path from " << start_city_name << " to " << end_city_name << ":\n";
            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << path[i];
                if (i < path.size() - 1) {
                    std::cout << " -> ";
                }
            }
            std::cout << "\nTotal distance: " << dists[end_node_id] << "\n";
        }
    }
};

int main() {
    Graph road_network;

    std::cout << "--- Initial Road Network Setup ---\n";
    // Add cities and roads to the network
    road_network.add_road("A", "B", 4, true); // Bidirectional
    road_network.add_road("A", "C", 2);      // Directed
    road_network.add_road("B", "E", 3);
    road_network.add_road("C", "D", 2);
    road_network.add_road("C", "F", 4, true); // Bidirectional
    road_network.add_road("D", "E", 3);
    road_network.add_road("D", "G", 1);
    road_network.add_road("E", "F", 1);
    road_network.add_road("F", "G", 1);
    std::cout << "--- Initial Setup Complete ---\n";

    // Find and display shortest paths
    road_network.find_shortest_path("A", "G");
    std::cout << "\n";
    road_network.find_shortest_path("A", "E");
    std::cout << "\n";
    road_network.find_shortest_path("G", "A"); // Example for an unreachable path in a directed graph
    std::cout << "\n";

    std::cout << "\n--- Simulating Traffic (Dynamic Weight Update) ---\n";
    // Simulate traffic by increasing distance on A->B road
    road_network.update_road_weight("A", "B", 10);
    std::cout << "Updated A to B road weight to 10 due to traffic.\n";
    road_network.find_shortest_path("A", "G"); // Re-run to see impact of traffic
    std::cout << "\n";

    std::cout << "\n--- Pathfinding with Non-existent Cities ---\n";
    road_network.find_shortest_path("A", "Z"); // Example for non-existent end city
    std::cout << "\n";
    road_network.find_shortest_path("X", "Y"); // Example for non-existent start city (and end)
    std::cout << "\n";

    return 0;
}

Route Finding System (Dijkstra's Algorithm)
This project implements a basic route-finding system using Dijkstra's algorithm in C++. It simulates a road network (graph) and calculates the shortest path between two specified cities, considering various road conditions.

🚀 Features
Graph Representation: Utilizes an Adjacency List to represent the road network efficiently.

City Management: Dynamically maps city names (strings) to unique integer IDs for graph processing.

Shortest Path Calculation: Implements Dijkstra's Algorithm to find the shortest path based on road weights (distances).

Bidirectional Roads: Supports defining roads that can be traveled in both directions with a single declaration.

Dynamic Edge Weights (Traffic Simulation): Allows updating road weights at runtime to simulate changing conditions like traffic.

Algorithm Trace: Provides a step-by-step console output of Dijkstra's algorithm's execution, showing node processing and distance updates, aiding in understanding the algorithm's flow.

Path Reconstruction: Reconstructs and displays the actual shortest path as a sequence of city names.

Error Handling: Basic handling for non-existent start/end cities.

💡 Core DSA Concepts Demonstrated
Graphs: Representation using Adjacency Lists.

Dijkstra's Algorithm: For finding the single-source shortest paths in a weighted, non-negative graph.

Priority Queue (std::priority_queue): Used to efficiently extract the minimum distance node in Dijkstra's algorithm.

Maps (std::map): For mapping string (city names) to integer IDs and vice-versa, allowing for human-readable input/output.

Vectors (std::vector): For dynamic arrays to store adjacency lists, distances, and path predecessors.

🛠️ How to Compile and Run
This project is written in C++ and can be compiled using a standard C++ compiler like g++.

Save the Code: Save the provided C++ code as main.cpp (or any .cpp extension).

Open Terminal: Navigate to the directory where you saved the file using your terminal or command prompt.

Compile: Use the g++ compiler to compile the code.

g++ main.cpp -o route_finder -std=c++11

(The -std=c++11 flag ensures C++11 features like std::tuple and range-based for loops are supported, though modern compilers might default to a newer standard.)

Run: Execute the compiled program.

./route_finder

🖥️ Example Output
--- Initial Road Network Setup ---
--- Initial Setup Complete ---
--- Dijkstra's Algorithm Trace ---
Starting from: A
Processing node: A (Current dist: 0)
  Updating path to B via A. New distance: 4
  Updating path to C via A. New distance: 2
Processing node: C (Current dist: 2)
  Updating path to D via C. New distance: 4
  Updating path to F via C. New distance: 6
Processing node: B (Current dist: 4)
  Updating path to E via B. New distance: 7
Processing node: D (Current dist: 4)
  Updating path to G via D. New distance: 5
  Considering edge D -> E (Weight: 3). No improvement.
Processing node: G (Current dist: 5)
--- Dijkstra's Algorithm Trace End ---

Shortest path from A to G:
A -> C -> D -> G
Total distance: 5

--- Dijkstra's Algorithm Trace ---
Starting from: A
Processing node: A (Current dist: 0)
  Updating path to B via A. New distance: 4
  Updating path to C via A. New distance: 2
Processing node: C (Current dist: 2)
  Updating path to D via C. New distance: 4
  Updating path to F via C. New distance: 6
Processing node: B (Current dist: 4)
  Updating path to E via B. New distance: 7
Processing node: D (Current dist: 4)
  Updating path to G via D. New distance: 5
  Considering edge D -> E (Weight: 3). No improvement.
Processing node: F (Current dist: 6)
  Considering edge F -> G (Weight: 1). No improvement.
Processing node: E (Current dist: 7)
  Considering edge E -> F (Weight: 1). No improvement.
--- Dijkstra's Algorithm Trace End ---

Shortest path from A to E:
A -> C -> D -> E
Total distance: 7

--- Dijkstra's Algorithm Trace ---
Starting from: G
Processing node: G (Current dist: 0)
--- Dijkstra's Algorithm Trace End ---
No path found from G to A


--- Simulating Traffic (Dynamic Weight Update) ---
Updated A to B road weight to 10 due to traffic.
--- Dijkstra's Algorithm Trace ---
Starting from: A
Processing node: A (Current dist: 0)
  Updating path to C via A. New distance: 2
  Updating path to B via A. New distance: 10
Processing node: C (Current dist: 2)
  Updating path to D via C. New distance: 4
  Updating path to F via C. New distance: 6
Processing node: D (Current dist: 4)
  Updating path to G via D. New distance: 5
  Considering edge D -> E (Weight: 3). No improvement.
Processing node: G (Current dist: 5)
--- Dijkstra's Algorithm Trace End ---

Shortest path from A to G:
A -> C -> D -> G
Total distance: 5


--- Pathfinding with Non-existent Cities ---
Error: End city 'Z' not found.

Error: Start city 'X' not found.

⏭️ Future Enhancements
This project can be further extended to incorporate more advanced features, making it a robust and comprehensive navigation system:

A* Search Algorithm: Implement A* search for more efficient pathfinding on larger graphs, especially when a heuristic (e.g., straight-line distance) is available.

Integration with Actual Map Data: Parse real-world geographical data (e.g., from OpenStreetMap, using libraries like RapidJSON for JSON parsing or TinyXML2 for XML) to build the graph, allowing for realistic mapping.

Graphical User Interface (GUI): Develop a visual interface (e.g., using SFML, SDL, Qt for C++, or a web-based approach with HTML/JavaScript Canvas) to:

Visually draw the map (cities and roads).

Highlight the nodes being processed and the final shortest path.

Allow interactive input of start/end points.

Provide controls for traffic simulation.

Finding the k-Shortest Paths: Implement algorithms like Yen's algorithm to find not just the shortest path, but also the second, third, and so on, shortest paths between two points.

Time-Dependent Routing: Incorporate more complex traffic models where travel times depend on the time of day.

Obstacle Avoidance: Extend to handle dynamic obstacles or blocked roads.
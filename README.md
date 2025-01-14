# Search Algorithms in MATLAB

## Description
This repository contains MATLAB implementations of classical graph search algorithms:
- Uniform Cost Search
- A* Algorithm
- Best-First Search
- Breadth-First Search
- Depth-First Search

These algorithms were implemented as part of an MSc course project on Artificial Intelligence & Machine Learning. The project demonstrates their application on two graph problems:
1. An example graph with edge costs and heuristic estimates.
2. A graph representing Romanian cities, where the goal is to find the shortest path to Bucharest.

## Repository Structure
- **Algorithm Scripts for the 1st example**:
  - `search_uniform_example1.m`: Implementation of Uniform Cost Search.
  - `search_astar_example1.m`: Implementation of the A* Algorithm.
  - `search_bestfirst_example1.m`: Implementation of Best-First Search.
  - `search_bfs_example1.m`: Implementation of Breadth-First Search.
  - `search_dfs_example1.m`: Implementation of Depth-First Search.
- **Algorithm Scripts for the 2nd example**:
  - `search_uniform_example2.m`: Implementation of Uniform Cost Search.
  - `search_astar_example2.m`: Implementation of the A* Algorithm.
  - `search_bestfirst_example2.m`: Implementation of Best-First Search.
  - `search_bfs_example2.m`: Implementation of Breadth-First Search.
  - `search_dfs_example2.m`: Implementation of Depth-First Search.
- **Unified Script**:
  - `search_all_example2.m`: Unified script to run any algorithm by specifying the method at runtime.

## How to Use
1. Clone the repository:
   ```bash
   git clone https://github.com/tSopermon/search-algorithms-matlab-aivc.git
   cd search-algorithms-matlab-aivc
2. Load the MATLAB environment.
3. To run a specific algorithm:
   ```matlab
   run('search_astar_example1.m');
4. To use the unified script:
   ```matlab
   run('search_all_example2.m');

## Features
- Blind search techniques: Breadth-First Search and Depth-First Search.
- Esge cost & Heuristic-based techniques: A*, Best-First Search, and Uniform Cost Search.
- Unified script for easy switching between algorithms.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact
For any inquiries or feedback, feel free to contact me at nikos.tsopanidis746@gmail.com.
```yaml

---

This organization ensures the repository is easy to navigate, well-documented, and professional. Let me know if you'd like to make further changes!

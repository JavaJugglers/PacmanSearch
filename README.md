# README for Pacman Pathfinding Project

## Introduction
This project involves developing a Pacman agent capable of navigating a maze to reach specific locations and efficiently collect food. The core challenge lies in implementing various search algorithms and applying them to different Pacman scenarios.

## Key Features and Implementations

### 1. General Search Algorithms
Implemented various general search algorithms in `search.py`. These include:
- Depth-First Search (DFS)
- Breadth-First Search (BFS)
- Uniform-Cost Search (UCS)
- A* Search

### 2. Custom Search Agents
Developed custom agents in `searchAgents.py` to utilize the search algorithms. This includes agents for different maze types and objectives, such as finding the shortest path or collecting all food.

### 3. Algorithm-Specific Features
- **DFS**: Implemented as a graph search to avoid revisiting states.
- **BFS**: Designed to find the least-cost solution.
- **UCS**: Adjusted for different cost functions.
- **A* Search**: Utilizes heuristics for efficient pathfinding.

### 4. Complex Problem Solving
- **Corners Problem**: Developed a heuristic to find the shortest path touching all four maze corners.
- **Food Search Problem**: Implemented a consistent heuristic for finding an optimal path to collect all food.

### 5. Suboptimal Search Strategies
Implemented a strategy for quickly finding reasonably good paths, even if not optimal.

## Testing and Validation
Extensively tested the algorithms and agents using provided test cases and mazes. Each algorithm was verified for efficiency and accuracy in various maze configurations and objectives.

## Usage
To run a specific search algorithm with the Pacman agent, use the following command format:
```bash
python pacman.py -l [MAZE_LAYOUT] -p SearchAgent -a fn=[ALGORITHM],additional_parameters
```
For example, to run BFS in mediumMaze:
```bash
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
```

# ExameCCM202
# Bi-Directional Rapidly-Exploring Random Tree (BiRRT)

This repository contains the implementation of the Bi-Directional Rapidly-Exploring Random Tree (BiRRT) algorithm for path planning in 2D environments with obstacles. It includes tools for simulation, performance analysis, and result visualization.

## Features
- Implementation of the BiRRT algorithm for path planning.
- Support for multiple test cases with different obstacle configurations.
- Monte Carlo simulation for performance analysis.
- Visualization of trees, obstacles, and planned paths.

## Project Files

### `monteCarloBiRRT.m`
Runs Monte Carlo simulations to evaluate the performance of the BiRRT algorithm in different scenarios. Features:
- Executes multiple runs of the algorithm.
- Collects statistics such as success rate, average path lengths, and costs of the paths found.
- Displays results in the console.

### `TreeNode.m`
Class representing the nodes of the trees used in the BiRRT algorithm. Each node stores:
- Position in the 2D space.
- Identification and parent indices.
- Function for extending towards a new position.

### `Tree.m`
Implements a tree data structure for the algorithm. Main methods:
- Node insertion.
- Finding the nearest node to a specified position.
- Reconstructing paths from the root to a target node.

### `BiRRT.m`
Class implementing the Bi-Directional RRT algorithm. Key features:
- Sampling from free space with collision checking.
- Alternating tree expansion in opposite directions.
- Detecting connection between the trees to form a complete path.
- Calculating cost and reconstructing the path from start to goal.

### `planPathRRT.m`
Script for:
- Planning paths using the BiRRT algorithm in test scenarios.
- Displaying animations of tree growth and path discovery.
- Saving visualizations in `png` or `eps` formats.

## Test Scenarios
The following test cases are supported:
- **Test Case A:** Single obstacle in the center of the space.
- **Test Case B:** Multiple obstacles in various positions.
- **Test Case C:** Dense obstacle configuration near the center.
- **Test Case D:** A line of obstacles across the middle of the space.

## Usage

### Monte Carlo Simulation
To run the Monte Carlo simulation, use the `monteCarloBiRRT.m` script:
```matlab
monteCarloBiRRT('a'); % Replace 'a' with 'b', 'c', or 'd' for other scenarios
```

### Path Planning and Visualization
To plan a path and visualize the process, use the `planPathRRT.m` script:
```matlab
planPathRRT('a'); % Replace 'a' with 'b', 'c', or 'd' for other scenarios
```

### Configurable Parameters
- **`maxIterations`**: Maximum number of iterations during planning.
- **`numRuns`**: Number of runs in the Monte Carlo simulation.
- **`goalBias`**: Probability of sampling towards the goal.
- **`delta`**: Step size for tree expansion.

## Results
The scripts generate statistics and graphs that include:
- Percentage of successful runs.
- Average length of the paths found.
- Average cost of the paths.

## Dependencies
- MATLAB R2020b or later.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Author
Developed by David Costa and Marcos Maximo.

## Acknowledgements
Based on research on Rapidly-Exploring Random Trees (RRT) and BiRRT algorithms for robotics and motion planning.


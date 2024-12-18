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

# RRT* Algorithm for Path Planning

This repository contains the implementation of the Rapidly-Exploring Random Tree Star (RRT*) algorithm for efficient path planning in 2D environments with obstacles. It includes functionality for planning paths, analyzing algorithm performance, and visualizing results.

## Features
- Implementation of the RRT* algorithm with cost optimization.
- Support for multiple test cases with different obstacle configurations.
- Monte Carlo simulation for performance evaluation.
- Visualization of the tree, obstacles, and planned paths.

## Project Files

### `TreeNode.m`
Class representing a node in the tree used by the RRT* algorithm. Each node contains:
- Position in 2D space.
- Index and parent index for path reconstruction.
- Accumulated cost from the root to the node.
- Methods for extending the node towards a new position.

### `Tree.m`
Class representing a tree data structure for the RRT* algorithm. Main methods include:
- Inserting nodes into the tree.
- Finding the nearest node to a specified position.
- Reconstructing the path from the root to a given goal.
- Selecting an optimal parent for a new node within a specified radius.
- Rewiring the tree to reduce path costs.

### `RRTStar.m`
Class implementing the RRT* algorithm. Key functionalities:
- Sampling from free space while avoiding collisions.
- Extending the tree by adding new nodes.
- Optimizing path costs by rewiring the tree.
- Calculating the total cost of paths.
- Generating a final path from source to goal.

### `planPathRRT.m`
Script for:
- Planning a path using the RRT* algorithm for a specified test case.
- Visualizing the tree growth and the final path.
- Saving the results as a plot in `png` or `eps` formats.

### `monteCarloRRTStar.m`
Script for running Monte Carlo simulations to evaluate the RRT* algorithm. Features:
- Multiple runs of the algorithm with random samples.
- Collection of statistics such as success rates, path lengths, and computation costs.
- Output of results to the console for analysis.

## Test Scenarios
The following test cases are included:
- **Test Case A:** Single obstacle at the center of the space.
- **Test Case B:** Multiple obstacles in varying configurations.
- **Test Case C:** Dense obstacle arrangement near the center.
- **Test Case D:** A line of obstacles dividing the space.

## Usage

### Monte Carlo Simulation
Run the Monte Carlo simulation using the script `monteCarloRRTStar.m`:
```matlab
monteCarloRRTStar('a'); % Replace 'a' with 'b', 'c', or 'd' for other scenarios
```

### Path Planning and Visualization
To plan a path and visualize the process, use the script `planPathRRT.m`:
```matlab
planPathRRT('a'); % Replace 'a' with 'b', 'c', or 'd' for other scenarios
```

### Configurable Parameters
- **`maxIterations`**: Maximum number of iterations for path planning.
- **`numRuns`**: Number of Monte Carlo simulation runs.
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


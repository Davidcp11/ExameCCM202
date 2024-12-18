# ExameCCM202
# Bi-Directional Rapidly-Exploring Random Tree (BiRRT) Path Planning

This repository implements the Bi-Directional Rapidly-Exploring Random Tree (BiRRT) algorithm for path planning in a 2D environment with obstacles. The repository provides tools for planning paths, evaluating the algorithm's performance through Monte Carlo simulations, and visualizing the results.

## Features
- Implementation of the BiRRT algorithm for path planning.
- Support for multiple test cases with different obstacle configurations.
- Monte Carlo evaluation for algorithm performance analysis.
- Visualization of trees, obstacles, and paths.

## File Descriptions

### `monteCarloBiRRT.m`
This script runs a Monte Carlo simulation to evaluate the BiRRT algorithm across different test cases. Key functionalities include:
- Running multiple iterations of the algorithm.
- Gathering statistics such as the percentage of successful runs, mean path lengths, and the cost of found paths.
- Printing results to the console for analysis.

### `TreeNode.m`
Defines a class for tree nodes used in the BiRRT algorithm. Each node stores:
- Position in the 2D space.
- Index and parent index for path reconstruction.
- Functions for extending the node towards a new position.

### `Tree.m`
Represents a tree structure used in the RRT algorithm. Key methods include:
- Inserting nodes into the tree.
- Finding the nearest node to a given position.
- Reconstructing paths from root to a specified node.

### `BiRRT.m`
Implements the Bi-Directional RRT algorithm. Key features:
- Supports sampling from free space with collision checking.
- Extends trees in alternating directions.
- Detects when the two trees connect to form a complete path.
- Calculates path costs and reconstructs paths from source to goal.

### `planPathRRT.m`
A script that:
- Plans a path using the BiRRT algorithm for a specified test case.
- Animates the process of tree growth and path discovery.
- Saves the resulting visualization in a user-specified format (`png` or `eps`).

## Test Cases
The following test cases are provided:
- **Test Case A:** Single obstacle at the center of the space.
- **Test Case B:** Multiple obstacles with varying positions.
- **Test Case C:** Dense configuration of obstacles near the center.
- **Test Case D:** A line of obstacles across the middle of the space.

## Usage
Use the planPathRRT.m script to visualize the planning process for a specific test case:
planPathRRT('a'); % Replace 'a' with 'b', 'c', or 'd' for other test cases
### Monte Carlo Simulation
Run the `monteCarloBiRRT.m` script with the desired test case:
```matlab
monteCarloBiRRT('a'); % Replace 'a' with 'b', 'c', or 'd' for other test cases


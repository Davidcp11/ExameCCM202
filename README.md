# ExameCCM202
# Path Planning Algorithms: RRT, RRT*, and BiRRT

This repository consolidates three key path planning algorithms: Rapidly-Exploring Random Tree (RRT), Rapidly-Exploring Random Tree Star (RRT*), and Bi-Directional Rapidly-Exploring Random Tree (BiRRT). These algorithms are implemented for planning paths in 2D environments with obstacles, providing tools for evaluation, visualization, and comparison of their features and performance.

---

## Algorithms Overview

### Rapidly-Exploring Random Tree (RRT)
- **Description**: The RRT algorithm incrementally builds a tree that explores the space by randomly sampling points and extending the tree towards those points. It is simple and efficient for high-dimensional spaces.
- **Advantages**:
  - Efficient exploration of large spaces.
  - Straightforward implementation.
  - Suitable for motion planning in dynamic environments.
- **Disadvantages**:
  - Does not guarantee optimality of the path.
  - Paths can be jagged and suboptimal.

### Rapidly-Exploring Random Tree Star (RRT*)
- **Description**: RRT* extends RRT by introducing optimization steps to rewire the tree and reduce the cost of paths. It guarantees asymptotic optimality, meaning the solution approaches the optimal path as computation time increases.
- **Advantages**:
  - Produces optimal paths given enough iterations.
  - Improves path quality through rewiring.
- **Disadvantages**:
  - Increased computational overhead compared to RRT.
  - Slower convergence in high-dimensional spaces.

### Bi-Directional Rapidly-Exploring Random Tree (BiRRT)
- **Description**: BiRRT grows two trees simultaneously, one from the source and another from the goal, attempting to connect them. This approach reduces the exploration time by focusing on the connection of the two trees.
- **Advantages**:
  - Faster pathfinding compared to single-tree RRT.
  - Suitable for narrow passages and challenging environments.
- **Disadvantages**:
  - Does not guarantee path optimality.
  - Requires careful tuning of parameters to balance the growth of both trees.

---

## Features
- Implementation of RRT, RRT*, and BiRRT algorithms.
- Support for multiple test cases with different obstacle configurations.
- Monte Carlo simulation for performance evaluation.
- Visualization tools for trees, obstacles, and planned paths.

---

## Project Files

### Core Components
- **`TreeNode.m`**: Defines the nodes used in the tree structure for all algorithms. Each node stores position, parent index, and cost information.
- **`Tree.m`**: Implements the tree structure, including methods for inserting nodes, finding the nearest node, reconstructing paths, and optimizing costs.

### Algorithm-Specific Components
- **`RRT.m`**: Implements the RRT algorithm with random sampling and path planning.
- **`RRTStar.m`**: Extends RRT with cost optimization and rewiring for improved paths.
- **`BiRRT.m`**: Implements the Bi-Directional RRT algorithm for simultaneous exploration from source and goal.

### Evaluation and Visualization
- **`planPathRRT.m`**: Plans a path using RRT and visualizes the result.
- **`planPathRRTStar.m`**: Plans a path using RRT* and visualizes the result.
- **`planPathBiRRT.m`**: Plans a path using BiRRT and visualizes the result.
- **`monteCarloRRT.m`**: Runs Monte Carlo simulations for RRT.
- **`monteCarloRRTStar.m`**: Runs Monte Carlo simulations for RRT*.
- **`monteCarloBiRRT.m`**: Runs Monte Carlo simulations for BiRRT.

---

## Test Scenarios
The following test cases are included:
- **Test Case A:** Single obstacle at the center of the space.
- **Test Case B:** Multiple obstacles in varying configurations.
- **Test Case C:** Dense obstacle arrangement near the center.
- **Test Case D:** A line of obstacles dividing the space.

---

## Usage

### Monte Carlo Simulation
Run Monte Carlo simulations for any algorithm:
```matlab
monteCarloRRT('a');       % RRT for Test Case A
monteCarloRRTStar('b');   % RRT* for Test Case B
monteCarloBiRRT('c');     % BiRRT for Test Case C
```

### Path Planning and Visualization
Plan and visualize paths using the respective planning scripts:
```matlab
planPathRRT('a');         % RRT for Test Case A
planPathRRTStar('b');     % RRT* for Test Case B
planPathBiRRT('c');       % BiRRT for Test Case C
```

### Configurable Parameters
- **`maxIterations`**: Maximum number of iterations for path planning.
- **`numRuns`**: Number of Monte Carlo simulation runs.
- **`goalBias`**: Probability of sampling towards the goal.
- **`delta`**: Step size for tree expansion.

---

## Results
Performance metrics and visualizations include:
- Percentage of successful runs.
- Average path lengths and costs.
- Standard deviations for path lengths and costs.
- Iterations to convergence.

---



## Dependencies
- MATLAB R2020b or later.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Author
Developed by David Costa and Marcos Maximo.

## Acknowledgements
Based on research on Rapidly-Exploring Random Trees (RRT) and BiRRT algorithms for robotics and motion planning.


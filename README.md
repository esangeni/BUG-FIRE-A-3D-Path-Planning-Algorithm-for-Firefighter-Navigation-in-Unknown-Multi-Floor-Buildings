# 3D BUG Algorithm for Multi-Floor Path Planning

## Overview

This repository presents a novel 3D BUG algorithm designed for autonomous navigation in unknown multi-floor indoor environments, with a specific focus on assisting firefighters. The algorithm addresses the unique challenges of three-dimensional path planning in complex building structures where prior map information is unavailable.

This implementation guarantees convergence for pedestrian agents or robots navigating to complex goals within multi-story buildings. While the resulting paths may not be optimal due to the absence of prior map information, the algorithm ensures that a viable path to the target is always found, regardless of the building's layout complexity.

The algorithm combines the classic BUG2 algorithm for planar route planning and a custom algorithm to find multi-floor paths to reach the goal floor which then we can implement BUG2. This adapted to work in three-dimensional spaces with multiple floors.

## Simulation Features

- **3D Path Planning**: Navigate through unknown multi-story buildings with complex floor layouts.
- **Obstacle Avoidance**: Efficiently maneuver around obstacles on each floor.
- **Floor Transitions**: Seamlessly plan paths that involve changing floors via staircases or elevators.
- **Monte Carlo Simulation**: Includes a robust testing framework to evaluate algorithm performance across various scenarios.
- **Visualization**: 3D plotting capabilities to visualize the environment, obstacles, and planned paths.

## Environment

The algorithm is designed and tested in a simulated environment (with Matlab) based on the Engineering Gateway building at the University of California, Irvine (UCI). The environment includes:

- Multiple floors (2nd, 3rd, and 4th floors)
- Complex room layouts and corridors
- Obstacles and impassable areas
- Inter-floor connections (staircases/elevators)

## Algorithm Description

The 3D BUG algorithm works as follows:

1. **Initialization**: Set start and goal points, which can be on different floors.
2. **If Start and Goal on the same floor**
2.1. **M-line Calculation**: Compute a 3D M-line connecting start and goal points.
2.2. **Path Planning**:
   - Move towards the goal along the M-line.
   - If an obstacle is encountered, circumnavigate it while keeping track of the closest point to the goal.
   - Return to the M-line at the closest point if a shorter path to the goal is found.
   - Handle floor transitions when necessary.
2.3. **Termination**: Reach the goal or determine that it's unreachable.
3. **If Start and Goal on different floor**
3.1. **Emergency Stairs Exploration Algorithm**: Survey algorithm for emergency signs and emergency stairs to connect floors and reach same floor as the goal.
3.2. **Then move to point 2.**

## Usage

Apart from downloading this repository, it also utilizes a Peter's Coke 2D BUG algorithm and some navigation functions from that repository. Please download it to run the simulations.
[Include instructions on how to run the algorithm, any dependencies, and example usage](https://petercorke.com/toolboxes/robotics-toolbox/)

## Monte Carlo Simulation

The repository includes a Monte Carlo simulation framework to test the algorithm's performance:

- Generates random start and goal points across different floors.
- Runs multiple iterations to assess success rate, path length, and execution time.
- Provides statistical analysis of the results.

## Visualization

The code includes 3D visualization tools to:

- Display the multi-floor environment.
- Show obstacles and floor layouts.
- Illustrate planned paths and algorithm execution.

## Results

The simulations allow us to play with two different parameters radius vision of the robot and the width of the vision cone of the robot. The simulations result in a 100% convergence in all tested scenarios. This way proposing an indoor algorithm that guarantees a path to the goal even though the initial information is just the start and final coordinates. No pre-map information is known by the agent.

## Future Work

- Extend this work to multi-wing floor types.
- Include positioning errors such as position drift.

## Contributors

Eudald Sangenis Rafart

## License

This project is released under the MIT License. You are free to use, modify, and distribute this work, provided that you give appropriate credit to the original author, Eudald Sangenis Rafart.

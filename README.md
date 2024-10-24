**README.md**

```markdown
# Motion Planning Code

Welcome to the **Motion Planning Code** repository! This project implements a motion planning and robotics simulation for a robotic manipulator (specifically, the Panda robot). It allows you to simulate picking, placing, stacking, and unstacking actions with cubes on a 3x3 grid. The code is designed to help understand motion planning algorithms and test various scenarios in robotics.

## Features

- **Simulate Motion Planning**: Run simulations with random or manually selected actions to test the motion planning capabilities.
- **Manual Testing**: Define custom states and action sequences to test specific scenarios.
- **Metrics and Analysis**: Collect and display metrics such as success rates, discrepancies, and effect checks.
- **Customizable Parameters**: Adjust the number of simulations, cubes, stack heights, and action choices.

## Getting Started

### Prerequisites

- [ROS (Robot Operating System)](http://www.ros.org/) (tested with ROS Melodic and Noetic)
- [MoveIt!](https://moveit.ros.org/) motion planning framework
- Panda robot MoveIt! configuration package (`franka_ros` and `panda_moveit_config`)
- C++ compiler supporting C++11 or higher
- Other dependencies as specified in your `package.xml` and `CMakeLists.txt`

### Installation

1. **Set up your ROS workspace** if you haven't already.

2. **Clone this repository** into your ROS workspace's `src` directory:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/yourusername/motion-planning-code.git
   ```

3. **Build the workspace**:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Source your workspace**:

   ```bash
   source devel/setup.bash
   ```

## Running the Simulation

To run the motion planning simulation, follow these steps:

1. **Launch the Panda robot MoveIt! demo**:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

2. **In a new terminal**, make sure to source your workspace and then run the motion planning node:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   rosrun motion_planning motion_planning_node
   ```

   **Note**: Replace `motion_planning` with your actual package name and `motion_planning_node` with the name of your executable if different.

**Ensure** that you have a ROS master running and that the necessary MoveIt! configurations are set up for the Panda robot.

## Documentation

- [Code Instructions](CodeInstructions.md): Detailed guide on how to run and test the motion planning and robotics simulation.

```


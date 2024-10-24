# README.md

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
- C++ compiler supporting C++11 or higher
- Other dependencies as specified in your `package.xml` and `CMakeLists.txt`

### Installation

1. Clone this repository into your ROS workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/yourusername/motion-planning-code.git
   ```

2. Build the package:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source your workspace:

   ```bash
   source devel/setup.bash
   ```

## Running the Simulation

To run the motion planning simulation, execute the following command:

```bash
roslaunch motion_planning_code panda_motion_planner.launch
```

**Note**: Ensure that you have a ROS master running and the necessary MoveIt! configurations set up for the Panda robot.

## Documentation

- [Code Instructions](CodeInstructions.md): Detailed guide on how to run and test the motion planning and robotics simulation.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss changes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---


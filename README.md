# Motion-Planning-Code
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

### Setup

To replicate the setup and run the motion planning code, follow these steps:

#### 1. Set Up Your ROS Workspace

If you haven't already set up a ROS workspace, follow the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to create one:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

#### 2. Clone the `motion_planning` Package

Navigate to the `src` directory of your workspace and clone this repository:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/motion_planning.git
```

**Note**: Replace `yourusername` with your GitHub username or the appropriate repository URL.

The directory structure should now look like this:

```
catkin_ws/
  src/
    motion_planning/
      src/
        motion_planning_node.cpp
      launch/
      CMakeLists.txt
      package.xml
      ... (other files)
```

#### 3. Ensure `panda_moveit_config` is Installed

The simulation relies on the `panda_moveit_config` package. If you don't have it installed, you can install it using:

```bash
sudo apt-get install ros-<distro>-panda-moveit-config
```

Replace `<distro>` with your ROS distribution, e.g., `melodic` or `noetic`.

Alternatively, you can clone it into your workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/panda_moveit_config.git
```

#### 4. Build the Workspace

After adding the `motion_planning` package (and `panda_moveit_config` if needed), build your workspace:

```bash
cd ~/catkin_ws
catkin_make
```

#### 5. Source Your Workspace

Source your workspace to overlay the new packages:

```bash
source ~/catkin_ws/devel/setup.bash
```

You should do this in every new terminal before running ROS commands.

## Running the Simulation

To run the motion planning simulation, follow these steps:

1. **Launch the Panda robot simulation with MoveIt!**:

   In a terminal, run:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

2. **In a new terminal, run the motion planning node**:

   Remember to source your workspace in this terminal:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

   Then run:

   ```bash
   rosrun motion_planning motion_planning_node
   ```

**Note**: Ensure that you have sourced your workspace in each terminal before running these commands. Also ensure you have a ROS master running and the necessary MoveIt! configurations set up for the Panda robot.

## Documentation

- [Code Instructions](CodeInstructions.md): Detailed guide on how to run and test the motion planning and robotics simulation.
---





















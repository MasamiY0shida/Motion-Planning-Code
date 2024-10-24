# CodeInstructions.md

# Code Instructions

This guide explains how to run and test the motion planning and robotics simulation. The main function in `panda_motion_planner.cpp` is where you can adjust parameters and define what you want to test.

## Table of Contents

- [Understanding the Main Function](#understanding-the-main-function)
- [Simulation vs. Manual Testing](#simulation-vs-manual-testing)
  - [Running Simulations](#running-simulations)
  - [Manual Testing](#manual-testing)
- [Adjusting Parameters](#adjusting-parameters)
  - [Simulation Parameters](#simulation-parameters)
  - [Execution Mode](#execution-mode)
  - [Action Selection](#action-selection)
- [Running the Code](#running-the-code)

## Understanding the Main Function

The `main` function is your starting point for running simulations or manual tests. Here's the structure of the `main` function:

```cpp
int main(int argc, char** argv)
{
    // ROS and MoveIt! initialization
    ros::init(argc, argv, "panda_motion_planner");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Instantiate the planner
    PandaMotionPlanner planner;

    // Parameters to adjust
    bool simulation_or_test = true; // true = simulation, false = manual testing
    bool execute = false; // true = execute actions, false = planning only

    if (simulation_or_test) {
        // Simulation parameters and execution
    } else {
        // Manual testing parameters and execution
    }

    ros::shutdown();
    return 0;
}
```

## Simulation vs. Manual Testing

### Running Simulations

When `simulation_or_test` is set to `true`, the code runs simulations with randomly generated states and actions or with specific actions you choose.

#### Adjusting Simulation Parameters

```cpp
// Set to true to run simulations
bool simulation_or_test = true;

// Whether to execute actions (true) or just plan (false)
bool execute = false;

// Simulation parameters
int num_simulations = 50; // Number of simulations to run
int max_cubes = 2;        // Number of cubes in the simulation
int max_height = 2;       // Maximum stack height per position

// Action selection
bool action_select = false; // false = random actions, true = manual action selection
int action_choice = 0;      // 0 = PICK, 1 = PLACE, 2 = STACK, 3 = UNSTACK
```

- **num_simulations**: The number of unique simulations you want to run.
- **max_cubes**: The total number of cubes available in the simulation.
- **max_height**: The maximum number of cubes that can be stacked at a single position.
- **action_select**: Choose whether to select actions manually or use random actions.
- **action_choice**: If `action_select` is `true`, specify which action to simulate:
  - `0`: PICK
  - `1`: PLACE
  - `2`: STACK
  - `3`: UNSTACK

#### Running the Simulations

The simulations collect data on motion planning success, precondition checks, and effect checks. Metrics and discrepancies are printed after the simulations are complete.

```cpp
// Run the simulations and get the results
auto [states, actions, results, resulting_states] = planner.runSimulations(
    num_simulations, max_cubes, max_height, execute, action_select, action_choice
);

// Analyze and print results
planner.runTaskPlanning(states, actions, results, resulting_states, precondition_results, effect_results);
planner.printResults(states, actions, results, precondition_results, effect_results, resulting_states);
```

### Manual Testing

When `simulation_or_test` is set to `false`, you can define a specific state and action sequence to test.

#### Defining Manual State and Actions

```cpp
// Set to false to perform manual testing
bool simulation_or_test = false;

// Whether to execute actions (true) or just plan (false)
bool execute = false;

// Manually define a state
PandaMotionPlanner::State manual_state = {
    0, 1, 0,
    0, 3, 0,
    1, 0, 1,
    1 // Gripper is holding a cube (cube_0)
};

// Manually define an action sequence
std::vector<PandaMotionPlanner::Action> manual_actions = {
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 0, 0),
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::UNSTACK, 4, 3),
    // ... (other actions)
};
```

- **manual_state**: Defines the initial positions and stack heights of cubes on the 3x3 grid. The last element indicates if the gripper is holding a cube (`1`) or not (`0`).
- **manual_actions**: A sequence of actions for the robot to perform.

#### Running the Manual Test

```cpp
// Run the manual state and action sequence
bool result = planner.run(manual_actions, manual_state, execute);

if (result) {
    ROS_INFO("Manual action sequence executed successfully.");
} else {
    ROS_ERROR("Manual action sequence failed.");
}
```

## Adjusting Parameters

### Simulation Parameters

- **Number of Simulations (`num_simulations`)**: Increase or decrease to run more or fewer simulations.
- **Number of Cubes (`max_cubes`)**: Set the total number of cubes in the simulation.
- **Maximum Stack Height (`max_height`)**: Limit how high the cubes can be stacked.

### Execution Mode

- **Execute Actions (`execute`)**:
  - `true`: The robot will execute the actions (requires more computation and visualization).
  - `false`: Only planning is performed without execution (faster).

### Action Selection

- **Action Select (`action_select`)**:
  - `false`: Actions are selected randomly.
  - `true`: You can manually select which action type to simulate.
- **Action Choice (`action_choice`)**:
  - `0`: PICK
  - `1`: PLACE
  - `2`: STACK
  - `3`: UNSTACK

## Running the Code

1. **Compile the Code**:

   ```bash
   catkin_make
   ```

2. **Source the Workspace**:

   ```bash
   source devel/setup.bash
   ```

3. **Run the Planner**:

   ```bash
   rosrun motion_planning_code panda_motion_planner
   ```

   **Note**: Replace `motion_planning_code` with your actual package name if different.

4. **Adjust Parameters**:

   - Open `panda_motion_planner.cpp`.
   - Modify the parameters in the `main` function as described above.
   - Recompile and rerun the code to see the effects of your changes.

## Additional Information

- **Cube Naming Convention**:
  - If the gripper is holding a cube (`g = 1`), it is named `cube_0`.
  - Cubes on the table are named starting from `cube_1`, assigned sequentially from the top-left to the bottom-right of the grid.
  - In stacks, higher-numbered cubes are on top of lower-numbered cubes.

- **Positions**:
  - The 3x3 grid positions are indexed from `0` to `8` (left to right, top to bottom).

- **Action Types**:
  - **PICK**: Pick up a cube from a position.
  - **PLACE**: Place a cube at a position.
  - **STACK**: Stack one cube onto another.
  - **UNSTACK**: Remove the top cube from a stack.

## Tips

- **Visualization**: If you have a visualization tool like RViz set up with MoveIt!, you can set `execute` to `true` to see the robot perform the actions.
- **Debugging**: Use `ROS_INFO`, `ROS_WARN`, and `ROS_ERROR` for debugging output.
- **Metrics Analysis**: After running simulations, review the printed metrics to analyze performance and discrepancies.

By following this guide, you should be able to run and test various scenarios in the motion planning simulation. Adjust the parameters as needed to explore different configurations and action sequences.
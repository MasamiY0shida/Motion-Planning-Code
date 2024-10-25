# Code Instructions

This guide explains how to run and test the motion planning and robotics simulation. All the **motion planning**, **checking against PDDL**, **comparing D<sub>f</sub> and D<sub>c</sub>**, as well as the **plan classification** parts are **self-contained** in the code. You only need to specify the parameters you want to run the simulation with (if you choose to run the simulation), or define your own manual test. The code handles everything else.

## Table of Contents

- [Understanding the Main Function](#understanding-the-main-function)
- [Simulation vs. Manual Testing](#simulation-vs-manual-testing)
  - [Running Simulations](#running-simulations)
  - [Manual Testing](#manual-testing)
- [Adjusting Parameters](#adjusting-parameters)
  - [Simulation Parameters](#simulation-parameters)
  - [Execution Mode](#execution-mode)
  - [Action Selection](#action-selection)
- [Analyzing the Results](#analyzing-the-results)
  - [Metrics Summary](#metrics-summary)
  - [Print Functions](#print-functions)
- [Running the Code](#running-the-code)
- [Additional Information](#additional-information)
- [Tips](#tips)
- [Support](#support)

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
    bool execute = false;           // true = execute actions, false = planning only

    if (simulation_or_test) {
        // Simulation parameters and execution
    } else {
        // Manual testing parameters and execution
    }

    ros::shutdown();
    return 0;
}
```

All the **motion planning**, **checking against PDDL**, **comparing D<sub>f</sub> and D<sub>c</sub>**, and **plan classification** are handled within the code above. You only need to adjust the parameters in the `main` function to run simulations or manual tests according to your requirements.

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

- **`simulation_or_test`**: Set to `true` to run simulations.
- **`execute`**: Choose whether to execute actions (`true`) or just perform planning (`false`).
- **`num_simulations`**: The number of unique simulations you want to run.
- **`max_cubes`**: The total number of cubes available in the simulation.
- **`max_height`**: The maximum number of cubes that can be stacked at a single position.
- **`action_select`**: Choose whether to select actions manually or use random actions.
- **`action_choice`**: If `action_select` is `true`, specify which action to simulate:
  - `0`: PICK
  - `1`: PLACE
  - `2`: STACK
  - `3`: UNSTACK

#### Running the Simulations

The simulations perform the following steps:

1. **Motion Planning**: Generates random initial states and actions (or uses specified actions) and attempts to perform motion planning for each action.
2. **Task-Level Feasibility Check (D<sub>f</sub>)**: Checks the preconditions and effects of each action against a PDDL representation of the task domain to determine task-level feasibility.
3. **Comparing D<sub>c</sub> and D<sub>f</sub>**: Compares the motion-level feasibility (D<sub>c</sub>) and task-level feasibility (D<sub>f</sub>) for each plan.
4. **Plan Classification**: Classifies plans based on the outcomes of D<sub>c</sub> and D<sub>f</sub>.

All these processes are handled internally by the code. You only need to set the parameters as shown above.

Here's how to run the simulations and analyze the results:

```cpp
// Start the timer
auto start_time = std::chrono::high_resolution_clock::now();

// Run the simulations and get the results
auto [states, actions, results, resulting_states] = planner.runSimulations(
    num_simulations, max_cubes, max_height, execute, action_select, action_choice
);

// Initialize vectors for precondition and effect results
std::vector<bool> precondition_results;
std::vector<bool> effect_results;

// Run the task planning and get the precondition and effect results
planner.runTaskPlanning(states, actions, results, resulting_states, precondition_results, effect_results);

// Collect discrepancies and compute metrics
// (Metrics calculation code here)

// Print the results
planner.printResults(states, actions, results, precondition_results, effect_results, resulting_states);

// Print discrepancies
planner.printDiscrepancies(discrepancies_preconditions, discrepancies_effects);

// Print the metrics (provides a comprehensive summary)
planner.printMetrics(total_plans, motion_successes, motion_failures,
                     precondition_successes, precondition_failures,
                     effect_successes, effect_failures,
                     black_list_type_1, white_list_type_1,
                     black_list_type_2, white_list_type_2);

// End the timer after simulations are complete
auto end_time = std::chrono::high_resolution_clock::now();

// Calculate the elapsed time
std::chrono::duration<double> elapsed_time = end_time - start_time;
ROS_INFO("Total time for %d simulations: %.2f seconds", num_simulations, elapsed_time.count());
```

The `printMetrics` function gives a **comprehensive summary** of the results in a short and neat table. It's usually the most important output to look at because it's concise, easy to read, and contains essential information such as:

- Number of plans sampled
- Plan classifications based on D<sub>c</sub> and D<sub>f</sub> outcomes
- Successes and failures of motion planning and task-level feasibility checks
- Time taken for simulations

This summary is displayed at the **end of the output**, making it easy to locate and review.

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

// State (with cube_ids) should look like this:
/*
Positions (indices):
[0]: []               // Position 0, no cubes
[1]: [cube_1]         // Position 1, cube_1 on table
[2]: []               // Position 2, no cubes
[3]: []               // Position 3, no cubes
[4]: [cube_4 (top), cube_3, cube_2 (bottom)] // Position 4, stack of 3 cubes
[5]: []               // Position 5, no cubes
[6]: [cube_5]         // Position 6, cube_5 on table
[7]: []               // Position 7, no cubes
[8]: [cube_6]         // Position 8, cube_6 on table
Gripper is holding cube_0
*/

// Manually define an action sequence
std::vector<PandaMotionPlanner::Action> manual_actions = {
    // Place cube_0 (from gripper) at position 0
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 0, 0),

    // Unstack cube_4 (topmost) from cube_3 at position 4
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::UNSTACK, 4, 3),

    // Place cube_4 at position 5
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 4, 5),

    // Unstack cube_3 (now topmost) from cube_2 at position 4
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::UNSTACK, 3, 2),

    // Place cube_3 at position 7
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 3, 7),

    // Pick cube_6 from position 8
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 6),

    // Place cube_6 at position 3
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 6, 3),

    // Pick cube_5 from position 6
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 5),

    // Place cube_5 at position 2
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 5, 2),

    // Pick cube_1 from position 1
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 1),

    // Stack cube_1 onto cube_0 at position 0
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::STACK, 1, 0),

    // Pick cube_2 from position 4 (now the only cube there)
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 2),

    // Stack cube_2 onto cube_5 at position 2
    PandaMotionPlanner::Action(PandaMotionPlanner::Action::STACK, 2, 5)
};

// Run the manual state and action sequence
bool result = planner.run(manual_actions, manual_state, execute);

if (result) {
    ROS_INFO("Manual action sequence executed successfully.");
} else {
    ROS_ERROR("Manual action sequence failed.");
}
```

- **`simulation_or_test`**: Set to `false` for manual testing.
- **`execute`**: Choose whether to execute actions (`true`) or just perform planning (`false`).
- **`manual_state`**: Define your custom initial state.
- **`manual_actions`**: Define your custom sequence of actions.

## Adjusting Parameters

### Simulation Parameters

You can adjust the parameters in the `main` function to customize your simulations:

- **`num_simulations`**: Set the number of simulations to run.
- **`max_cubes`**: Specify the total number of cubes.
- **`max_height`**: Define the maximum stack height per position.

### Execution Mode

- **`execute`**:
  - Set to `true` to execute actions and visualize them (requires more computation and visualization tools like RViz).
  - Set to `false` to only perform planning without execution (faster).

### Action Selection

- **`action_select`**:
  - Set to `false` to use random actions in simulations.
  - Set to `true` to manually select the action type for simulations.
- **`action_choice`**:
  - If `action_select` is `true`, specify the action type:
    - `0`: PICK
    - `1`: PLACE
    - `2`: STACK
    - `3`: UNSTACK

## Analyzing the Results

### Metrics Summary

After running simulations, the code provides a **comprehensive summary** of the results using the `printMetrics` function. This function outputs a neat table containing:

- **Total number of plans generated**
- **Successes and failures of motion planning (D<sub>c</sub>)**
- **Successes and failures of task-level feasibility checks (D<sub>f</sub>)**
- **Successes and failures of effect checks** (when both D<sub>c</sub> and D<sub>f</sub> succeed)
- **Plan classifications** based on D<sub>c</sub> and D<sub>f</sub> outcomes
- **Time taken** for the simulations

The metrics summary is displayed at the **end of the output**, making it easy to locate and review. This summary provides a quick and comprehensive overview of the simulation results and is usually the most important part to look at.

### Print Functions

The code includes several print functions to display detailed information:

- **`printResults`**: Displays detailed results for each simulation, including initial states, actions, motion planning results, precondition checks, and effect checks.
- **`printSimpleResults`**: Provides a concise version of the results.
- **`printDiscrepancies`**: Highlights any discrepancies between motion planning and task-level feasibility checks.
- **`printMetrics`**: Outputs the comprehensive summary of the results (as described above).

## Running the Code

1. **Compile the Code**:

   ```bash
   catkin_make
   ```

2. **Source the Workspace**:

   ```bash
   source devel/setup.bash
   ```

3. **Launch the Panda Robot Simulation with MoveIt!**:

   In a new terminal (remember to source your workspace), run:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

4. **Run the Motion Planning Node**:

   In another terminal (source your workspace), run:

   ```bash
   rosrun motion_planning motion_planning_node
   ```

   **Note**: The package is called `motion_planning`, and the node is `motion_planning_node`. Ensure your directory structure is as follows:

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

5. **Adjust Parameters**:

   - Open `motion_planning_node.cpp`.
   - Modify the parameters in the `main` function as described above.
   - Recompile (`catkin_make`) and rerun the code to see the effects of your changes.

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
- **Metrics Analysis**: Focus on the output of the `printMetrics` function for a quick and comprehensive understanding of the simulation results.

## Support

If you encounter issues or have questions, please refer to the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) for guidance on setting up your environment, creating packages, and working with nodes. You can also open an issue on the GitHub repository or contact the maintainer.

---

By following this guide, you can easily adjust the simulation parameters or define manual tests, run the motion planning simulation, and analyze the results. The `printMetrics` function provides a concise and comprehensive summary, which is especially useful for reviewing the overall performance and outcomes of your simulations.
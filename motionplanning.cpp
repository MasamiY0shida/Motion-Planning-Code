#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shape_operations.h>
#include <vector>
#include <cstdlib>  // For rand() and srand()
#include <ctime>    // For time()
#include <chrono> 
#include <algorithm>  // For std::min
#include <random>     // For better random number generation
#include <numeric>  // For std::iota
#include <set>
#include <sstream>
#include <string>

static const std::string ARM_PLANNING_GROUP = "panda_arm";
static const std::string GRIPPER_PLANNING_GROUP = "panda_hand";

class PandaMotionPlanner
{
public:
  PandaMotionPlanner()
    : arm_move_group(ARM_PLANNING_GROUP)
    , gripper_move_group(GRIPPER_PLANNING_GROUP)
    , visual_tools("panda_link0")
    , planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"))
{
    initializeMoveGroup();
    initializeVisualTools();
    initializeBasePositions();
}

void initializeMoveGroup() {
    constexpr double PLANNING_TIME = 10.0;
    arm_move_group.setPlanningTime(PLANNING_TIME);
}

void initializeVisualTools() {
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
}


  void initializeBasePositions()
    {
        base_positions = {
            {-0.15, -0.15, 0.425},
            {-0.15, 0, 0.425},
            {-0.15, 0.15, 0.425},
            {0, -0.15, 0.425},
            {0, 0, 0.425},
            {0, 0.15, 0.425},
            {0.15, -0.15, 0.425},
            {0.15, 0, 0.425},
            {0.15, 0.15, 0.425}
        };
    }

  void attachObject(const std::string& object_id, const std::string& link_name) {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    attached_object.object.id = object_id;
    attached_object.object.operation = attached_object.object.ADD;

    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    ROS_INFO("Attached object '%s' to link '%s'", object_id.c_str(), link_name.c_str());
}

void detachObject(const std::string& object_id) {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object.id = object_id;
    attached_object.object.operation = attached_object.object.REMOVE;

    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    ROS_INFO("Detached object '%s'", object_id.c_str());
}

  void allowGripperCubeCollision(const std::string& target_cube_id)
{
    // Use the existing class-level planning_scene_monitor_ instance
    if (!planning_scene_monitor_) {
        ROS_ERROR("Planning scene monitor is not initialized.");
        return;
    }

    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    collision_detection::AllowedCollisionMatrix& acm = ps->getAllowedCollisionMatrixNonConst();

    std::vector<std::string> gripper_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};

    // First, disallow collision between gripper and all cubes
    std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();
    for (const auto& id : object_ids) {
        if (id.find("small_cube") != std::string::npos) {
            for (const auto& link : gripper_links) {
                acm.setEntry(id, link, false);
            }
        }
    }

    // Then, allow collision only for the target cube
    for (const auto& link : gripper_links) {
        acm.setEntry(target_cube_id, link, true);
    }

    moveit_msgs::PlanningScene planning_scene_msg;
    ps->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    ROS_INFO("Allowed collision between gripper and cube: %s", target_cube_id.c_str());
}


void disallowAllGripperCubeCollisions()
{
    // Use the existing class-level planning_scene_monitor_ instance
    if (!planning_scene_monitor_) {
        ROS_ERROR("Planning scene monitor is not initialized.");
        return;
    }

    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    collision_detection::AllowedCollisionMatrix& acm = ps->getAllowedCollisionMatrixNonConst();

    std::vector<std::string> gripper_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};
    std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();

    for (const auto& id : object_ids) {
        if (id.find("small_cube") != std::string::npos) {
            for (const auto& link : gripper_links) {
                acm.setEntry(id, link, false);
            }
        }
    }

    moveit_msgs::PlanningScene planning_scene_msg;
    ps->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    ROS_INFO("Disallowed collisions between gripper and all cubes");
}


void allowCubeCollisions(bool allow = true)
{
    // Use the existing class-level planning_scene_monitor_ instance
    if (!planning_scene_monitor_) {
        ROS_ERROR("Planning scene monitor is not initialized.");
        return;
    }

    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    collision_detection::AllowedCollisionMatrix& acm = ps->getAllowedCollisionMatrixNonConst();

    std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();

    for (const auto& id : object_ids) {
        if (id.find("small_cube") != std::string::npos) {
            acm.setEntry(id, "table1", allow);  // Set collision with table1
            acm.setEntry(id, "table2", allow);  // Set collision with table2
        }
    }

    moveit_msgs::PlanningScene planning_scene_msg;
    ps->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    ROS_INFO("%s collision between cubes and tables", allow ? "Allowed" : "Disallowed");
}

void allowSpecificCubeCollision(const std::string& cube_id1, const std::string& cube_id2, bool allow = true)
{
    // Use the existing class-level planning_scene_monitor_ instance
    if (!planning_scene_monitor_) {
        ROS_ERROR("Planning scene monitor is not initialized.");
        return;
    }

    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    collision_detection::AllowedCollisionMatrix& acm = ps->getAllowedCollisionMatrixNonConst();

    acm.setEntry(cube_id1, cube_id2, allow);

    moveit_msgs::PlanningScene planning_scene_msg;
    ps->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    ROS_INFO("%s collision between %s and %s", allow ? "Allowed" : "Disallowed", cube_id1.c_str(), cube_id2.c_str());
}


  struct GoalPositions {
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose goal_pose1;
    geometry_msgs::Pose goal_pose2;
    geometry_msgs::Pose table_center_pose;
};

geometry_msgs::Pose getObjectPose(const std::string& object_id)
{
    std::vector<std::string> object_ids;
    object_ids.push_back(object_id);
    std::map<std::string, geometry_msgs::Pose> object_poses = planning_scene_interface.getObjectPoses(object_ids);
    
    if (object_poses.find(object_id) != object_poses.end()) {
        return object_poses[object_id];
    } else {
        ROS_ERROR("Object '%s' not found in the planning scene", object_id.c_str());
        return geometry_msgs::Pose();
    }
}

std::vector<std::pair<std::string, geometry_msgs::Pose>> getAllCubePoses()
{
    std::vector<std::pair<std::string, geometry_msgs::Pose>> cube_poses;
    std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();
    
    for (const auto& id : object_ids) {
        if (id.find("small_cube") != std::string::npos) {
            geometry_msgs::Pose pose = getObjectPose(id);
            if (pose.position.x != 0 || pose.position.y != 0 || pose.position.z != 0) {
                cube_poses.push_back(std::make_pair(id, pose));
            }
        }
    }
    
    return cube_poses;
}

GoalPositions defineGoalPositions()
{
    GoalPositions positions;
    
    // Get the current pose of the end-effector
    positions.start_pose = arm_move_group.getCurrentPose().pose;

    // Define table center pose
    positions.table_center_pose = positions.start_pose;
    positions.table_center_pose.position.x += 0.225;  // 22.5cm forward

    return positions;
}

geometry_msgs::Pose calculateIntermediatePosition(const geometry_msgs::Pose& target_pose, double height_offset = 0.15)
{
    geometry_msgs::Pose intermediate_pose = target_pose;
    intermediate_pose.position.z += height_offset;
    
    // Set a specific orientation that points the gripper downward
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI/4);  // Roll, Pitch, Yaw
    intermediate_pose.orientation = tf2::toMsg(q);
    
    return intermediate_pose;
}


bool moveGripper(bool open, bool execute = true)
{
    // Set joint values for opening or closing the gripper
    std::vector<double> gripper_joint_values;
    if (open) {
        gripper_joint_values = {0.04, 0.04};  // Open gripper
    } else {
        gripper_joint_values = {0.025, 0.025};  // Close gripper
    }

    // Set the target joint values for the gripper
    gripper_move_group.setJointValueTarget(gripper_joint_values);

    // Reduce planning time and attempts for faster planning
    gripper_move_group.setPlanningTime(1.0);  // Reduced planning time
    gripper_move_group.setNumPlanningAttempts(2);  // Fewer planning attempts

    // Increase velocity and acceleration scaling factors to speed up execution
    gripper_move_group.setMaxVelocityScalingFactor(1.0);  // Max speed
    gripper_move_group.setMaxAccelerationScalingFactor(1.0);  // Max acceleration

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success = (gripper_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Gripper plan succeeded. Executing...");

        if (execute) {
            // Execute the gripper motion
            gripper_move_group.execute(gripper_plan);

        } else {
            // Skip execution
            ROS_INFO("Skipped execution");

            // Update the robot's internal state with the final joint positions from the plan
            updateInternalGripperState(gripper_plan);

        }
        // Optionally, if you want a visual prompt, uncomment the following line
        // visual_tools.prompt("Press 'next' to " + std::string(open ? "open" : "close") + " the gripper");

        ROS_INFO("Gripper %s executed successfully.", open ? "open" : "close");
    } else {
        ROS_ERROR("Gripper planning failed");
    }

    return success;
}

bool moveToPosition(const geometry_msgs::Pose& target_pose, const std::string& action_description, bool execute = true) {
    std::string end_effector_link = arm_move_group.getEndEffectorLink();
    ROS_INFO("End effector link: %s", end_effector_link.c_str());

    // Set the target pose
    arm_move_group.setPoseTarget(target_pose);

    // Set planning parameters
    arm_move_group.setPlanningTime(1.5);
    arm_move_group.setNumPlanningAttempts(3);

    // Set the planner ID
    arm_move_group.setPlannerId("RRTConnectkConfigDefault");

    // Set velocity and acceleration scaling factors
    arm_move_group.setMaxVelocityScalingFactor(0.8);
    arm_move_group.setMaxAccelerationScalingFactor(0.8);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool planning_success = (arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (planning_success) {
        ROS_INFO("Motion plan succeeded.");

        // Visualize the plan
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(
            my_plan.trajectory_,
            arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP)
        );
        visual_tools.trigger();

        if (execute) {
            ROS_INFO("Executing the plan...");
            moveit::core::MoveItErrorCode execution_result = arm_move_group.execute(my_plan);
            if (execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_ERROR("Execution failed.");
                return false;
            }
        } else {
            ROS_INFO("Execution skipped as per the input parameter.");

            // Update the robot's internal state with the final joint positions from the plan
            updateInternalRobotState(my_plan);
        }
    } else {
        ROS_ERROR("Failed to plan movement to target pose");
        return false;
    }
    return true;
}

void updateInternalGripperState(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // Get the last point in the trajectory
    if (plan.trajectory_.joint_trajectory.points.empty()) {
        ROS_WARN("Gripper planned trajectory is empty. Cannot update internal gripper state.");
        return;
    }

    const trajectory_msgs::JointTrajectoryPoint& last_point = plan.trajectory_.joint_trajectory.points.back();

    // Get the current state of the robot
    moveit::core::RobotStatePtr current_state = gripper_move_group.getCurrentState();

    // Update the joint positions
    const std::vector<std::string>& joint_names = plan.trajectory_.joint_trajectory.joint_names;
    const std::vector<double>& joint_positions = last_point.positions;

    if (joint_names.size() != joint_positions.size()) {
        ROS_ERROR("Mismatch between gripper joint names and joint positions size.");
        return;
    }

    // Set the joint positions in the current state
    current_state->setVariablePositions(joint_names, joint_positions);

    // Update the MoveGroup's internal state
    gripper_move_group.setStartState(*current_state);
}


void updateInternalRobotState(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // Get the last point in the trajectory
    if (plan.trajectory_.joint_trajectory.points.empty()) {
        ROS_WARN("Planned trajectory is empty. Cannot update internal robot state.");
        return;
    }

    const trajectory_msgs::JointTrajectoryPoint& last_point = plan.trajectory_.joint_trajectory.points.back();

    // Get the current state of the robot
    moveit::core::RobotStatePtr current_state = arm_move_group.getCurrentState();

    // Update the joint positions
    const std::vector<std::string>& joint_names = plan.trajectory_.joint_trajectory.joint_names;
    const std::vector<double>& joint_positions = last_point.positions;

    if (joint_names.size() != joint_positions.size()) {
        ROS_ERROR("Mismatch between joint names and joint positions size.");
        return;
    }

    // Set the joint positions in the current state
    current_state->setVariablePositions(joint_names, joint_positions);

    // Update the MoveGroup's internal state
    arm_move_group.setStartState(*current_state);
}




bool pickCube(const std::string& cube_id, const geometry_msgs::Pose& pick_pose, bool execute)
{
    // Calculate g1 (15 cm above pick_pose)
    geometry_msgs::Pose g1 = calculateIntermediatePosition(pick_pose);

    // Move to g1
    if (!moveToPosition(g1, "move to position above " + cube_id, execute)) {
        ROS_ERROR("FAILED TO MOVE TO POSITION ABOVE %s FOR PICKING.", cube_id.c_str());
        return false;
    }

    // Lower to pick position
    geometry_msgs::Pose lower_pose = pick_pose;
    lower_pose.position.z = pick_pose.position.z + 0.1;  // Slightly above the cube to account for gripper size
    if (!moveToPosition(lower_pose, "lower to pick " + cube_id, execute)) {
        ROS_ERROR("FAILED TO LOWER TO PICK %s.", cube_id.c_str());
        return false;
    }

    // Gripper actions
    if (!moveGripper(true, execute)) {  // Open gripper
        ROS_ERROR("FAILED TO OPEN GRIPPER FOR PICKING %s.", cube_id.c_str());
        return false;
    }

    // visual_tools.prompt("Press 'next' to close gripper and pick " + cube_id);

    if (!moveGripper(false, execute)) {  // Close gripper
        ROS_ERROR("FAILED TO CLOSE GRIPPER FOR PICKING %s.", cube_id.c_str());
        return false;
    }

    attachObject(cube_id, "panda_hand");

    // Move back to g1
    if (!moveToPosition(g1, "lift " + cube_id, execute)) {
        ROS_ERROR("FAILED TO LIFT %s AFTER PICKING.", cube_id.c_str());
        return false;
    }

    ROS_INFO("PICK OPERATION FOR %s SUCCEEDED.", cube_id.c_str());
    return true;
}

bool placeCube(const std::string& cube_id, const geometry_msgs::Pose& place_pose, bool execute)
{
    // Calculate g2 (15 cm above place_pose)
    geometry_msgs::Pose g2 = calculateIntermediatePosition(place_pose);

    // Move to g2
    if (!moveToPosition(g2, "move to position above placement for " + cube_id, execute)) {
        ROS_ERROR("FAILED TO MOVE TO POSITION ABOVE %s FOR PLACING.", cube_id.c_str());
        return false;
    }

    // Lower to place position
    geometry_msgs::Pose lower_pose = place_pose;
    lower_pose.position.z = place_pose.position.z + 0.12;  // Slightly above the target position
    if (!moveToPosition(lower_pose, "lower to place " + cube_id, execute)) {
        ROS_ERROR("FAILED TO LOWER TO PLACE %s.", cube_id.c_str());
        return false;
    }

    // Gripper actions
    // visual_tools.prompt("Press 'next' to open gripper and release " + cube_id);

    detachObject(cube_id);
    moveGripper(true, execute);

    // Move back to g2
    if (!moveToPosition(g2, "lift after placing " + cube_id, execute)) {
        ROS_ERROR("FAILED TO LIFT AFTER PLACING %s.", cube_id.c_str());
        return false;
    }

    ROS_INFO("PLACE OPERATION FOR %s SUCCEEDED.", cube_id.c_str());
    return true;
}


struct Action {
    enum Type { PICK, PLACE, STACK, UNSTACK };
    Type type;
    int cube_index1;       // For PICK, PLACE, STACK (cube to move), and UNSTACK (cube to move)
    int cube_index2;  // For STACK (cube to stack on) or PLACE (position to place the cube)
    int position_index;    // For UNSTACK (position to place the cube after unstacking)

    // Constructor for actions that involve only one cube (e.g., PICK)
    Action(Type t, int index1) : type(t), cube_index1(index1), cube_index2(-1), position_index(-1) {}

    // Constructor for actions that involve two inputs (e.g., PLACE and STACK)
    Action(Type t, int index1, int index2_or_pos) : type(t), cube_index1(index1), cube_index2(index2_or_pos), position_index(-1) {}

    // Constructor for UNSTACK (which takes cube_index1, cube_index2, and position_index)
    Action(Type t, int index1, int index2, int pos) : type(t), cube_index1(index1), cube_index2(index2), position_index(pos) {}
};

typedef std::vector<int> State;
std::vector<int> dynamic_state;


bool run(const std::vector<Action>& action_sequence, const State& initial_state, bool execute) {
    try {
        // Setup and initialization
        ROS_INFO("Adding collision objects...");
        addCollisionObjects(initial_state);
        allowCubeCollisions(true);
        
        GoalPositions positions = defineGoalPositions();
        dynamic_state = initial_state;
        
        // Get the initial pose of the gripper
        geometry_msgs::Pose initial_gripper_pose = arm_move_group.getCurrentPose().pose;

        // Get all cube poses
        ROS_INFO("Getting all cube poses...");
        std::vector<std::pair<std::string, geometry_msgs::Pose>> cube_poses = getAllCubePoses();

        std::string current_cube_id;
        geometry_msgs::Pose current_cube_pose;
        int current_pick_position = -1;
        bool gripper_has_cube = dynamic_state[9] == 1;

        // Initialize gripper state
        if (gripper_has_cube) {
            current_cube_id = "small_cube0";  // Using the new naming convention
            allowGripperCubeCollision(current_cube_id);

            
            // Close the gripper
            ROS_INFO("Closing the gripper with cube %s...", current_cube_id.c_str());
            if (!moveGripper(false, execute)) {
                return handleFailure(initial_gripper_pose, current_cube_id, true, execute);
            }

            // Attach the cube to the gripper
            attachObject(current_cube_id, "panda_hand");  // Assuming "panda_hand" is the link name for the gripper

            // Allow collision between this cube and the gripper
            allowGripperCubeCollision(current_cube_id);

            // Disallow collisions between this cube and all other cubes
            for (const auto& cube : cube_poses) {
                if (cube.first != current_cube_id) {
                    allowSpecificCubeCollision(current_cube_id, cube.first, false);
                }
            }

            ROS_INFO("Gripper initialized with cube %s attached", current_cube_id.c_str());
        } else {
            // Open the gripper
            ROS_INFO("Opening the gripper...");
            if (!moveGripper(true, execute)) {
                return handleFailure(initial_gripper_pose, "", true, execute);
            }

            // Disallow all gripper-cube collisions at the start if gripper is empty
            disallowAllGripperCubeCollisions();

            ROS_INFO("Gripper initialized in open position without cube");
        }

        for (const auto& action : action_sequence) {
            visual_tools.prompt("Press 'next' to plan the next action");
            ROS_INFO("Processing action: %d", action.type);
            bool action_success = false;
            switch (action.type) {
                case Action::PICK:
                    action_success = handlePickAction(action, dynamic_state, positions, cube_poses, current_cube_id, current_cube_pose, current_pick_position, gripper_has_cube, execute);
                    break;
                case Action::PLACE:
                    action_success = handlePlaceAction(action, dynamic_state, positions, cube_poses, current_cube_id, current_pick_position, gripper_has_cube, execute);
                    break;
                case Action::STACK:
                    action_success = handleStackAction(action, dynamic_state, positions, cube_poses, current_cube_id, current_pick_position, gripper_has_cube, execute);
                    break;
                case Action::UNSTACK:
                    action_success = handleUnstackAction(action, dynamic_state, positions, cube_poses, current_cube_id, gripper_has_cube, execute);
                    break;
                default:
                    ROS_ERROR("Unknown action type");
                    return handleFailure(initial_gripper_pose, current_cube_id, true, execute);
            }

            if (!action_success) {
                ROS_ERROR("Action failed: stopping sequence");
                return handleFailure(initial_gripper_pose, current_cube_id, true, execute);
            }

            // Print the current dynamic state for debugging
            ROS_INFO("Current dynamic state:");
            for (size_t i = 0; i < 9; ++i) {
                ROS_INFO("Position %zu: %d cubes", i, dynamic_state[i]);
            }
            ROS_INFO("Gripper state: %s", gripper_has_cube ? "Holding cube" : "Empty");
        }

        // Return to starting position
        ROS_INFO("Returning to starting position...");
        if (!moveToPosition(positions.start_pose, "return to the starting position", execute)) {
            return handleFailure(initial_gripper_pose, current_cube_id, true, execute);
        }
        
        visual_tools.prompt("Press 'next' to end the demo");

        return handleSuccess(current_cube_id, initial_gripper_pose, execute);

    } catch (const std::exception &e) {
        ROS_ERROR("Exception caught in run function: %s", e.what());
        return handleFailure(geometry_msgs::Pose(), "", true, execute);
    } catch (...) {
        ROS_ERROR("Unknown exception caught in run function");
        return handleFailure(geometry_msgs::Pose(), "", true, execute);
    }
}

bool handleFailure(const geometry_msgs::Pose& initial_gripper_pose, const std::string& current_cube_id, bool cleanup, bool execute)
{
    // Move gripper back to its original starting position and open it
    ROS_INFO("Returning to initial position and opening gripper due to failure.");

    // If gripper is holding a cube, detach it
    if (!current_cube_id.empty()) {
        detachObject(current_cube_id);  // Detach the cube from the gripper
        ROS_INFO("Detached cube %s from gripper", current_cube_id.c_str());
    }

    // Disallow all collisions between the gripper and cubes
    disallowAllGripperCubeCollisions();

    // Move gripper back to the initial position
    moveToPosition(initial_gripper_pose, "return to initial position", execute);
    
    // Open gripper before ending
    moveGripper(true, execute);

    ROS_ERROR("PLANNING FAILED");

    ROS_INFO("Clearing planning scene...");
    planning_scene_interface.clear();
    ROS_INFO("Planning scene cleared.");


    return false;
}

bool handleSuccess(std::string& current_cube_id, const geometry_msgs::Pose& initial_gripper_pose, bool execute)
{
    // Check if the gripper is holding a cube
    if (!current_cube_id.empty()) {
        // Detach the cube from the gripper
        detachObject(current_cube_id);
        ROS_INFO("Detached cube %s from gripper", current_cube_id.c_str());
        current_cube_id.clear();  // Clear the current cube ID
    }
    
    // Open gripper before ending
    moveGripper(true, execute);

    // Clear all collision objects from the planning scene
    ROS_INFO("Clearing planning scene...");
    
    planning_scene_interface.clear();

    ROS_INFO("Planning scene cleared.");

    
    ROS_INFO("PLANNING SUCCEEDED");
    return true;
}




bool handlePickAction(const Action& action, std::vector<int>& dynamic_state, GoalPositions& positions, std::vector<std::pair<std::string, geometry_msgs::Pose>>& cube_poses, std::string& current_cube_id, geometry_msgs::Pose& current_cube_pose, int& current_pick_position, bool& gripper_has_cube, bool execute) {
    if (gripper_has_cube) {
        ROS_ERROR("CANNOT PICK A CUBE. GRIPPER IS ALREADY HOLDING A CUBE.");
        return false;
    }
    if (action.cube_index1 >= cube_poses.size()) {
        ROS_ERROR("INVALID CUBE INDEX: %d", action.cube_index1);
        return false;
    }
    current_cube_id = cube_poses[action.cube_index1].first;
    current_cube_pose = cube_poses[action.cube_index1].second;

    // Initialize current_pick_position to -1
    current_pick_position = -1;

    // Find the position index of the cube being picked
    for (size_t i = 0; i < base_positions.size(); ++i) {
        if (std::abs(current_cube_pose.position.x - (positions.table_center_pose.position.x + base_positions[i].x)) < 0.01 &&
            std::abs(current_cube_pose.position.y - (positions.table_center_pose.position.y + base_positions[i].y)) < 0.01) {
            current_pick_position = i;
            break;
        }
    }

    if (current_pick_position == -1) {
        ROS_ERROR("Could not find the position index of the cube being picked.");
        return false;
    }

    // Add the restriction: The cube must be the only one at its position
    if (dynamic_state[current_pick_position] != 1) {
        ROS_ERROR("Cannot pick cube %s at position %d. There are other cubes at this position.", current_cube_id.c_str(), current_pick_position);
        return false;
    }

    // Allow collision only with the cube we're about to pick
    allowGripperCubeCollision(current_cube_id);

    // Disallow collisions between the picked cube and all other cubes
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, false);
        }
    }

    geometry_msgs::Pose pick_pose = current_cube_pose;
    pick_pose.position.z += 0.02;  // Slightly above the cube
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI / 4);  // Roll, Pitch, Yaw
    pick_pose.orientation = tf2::toMsg(q);

    // Attempt to pick the cube, return false if it fails
    if (!pickCube(current_cube_id, pick_pose, execute)) {
        return false;
    }

    // Update dynamic state and gripper status
    dynamic_state[current_pick_position]--;  // Remove the cube from the position
    gripper_has_cube = true;
    dynamic_state[9] = 1;  // Update gripper state
    if (!execute) {
        planning_scene_interface.clear();
        ros::Duration(0.001).sleep();
        addCollisionObjects(dynamic_state);
    }
    ROS_INFO("PICK ACTION SUCCEEDED.");
    return true;
}



bool handlePlaceAction(const Action& action, std::vector<int>& dynamic_state, GoalPositions& positions, std::vector<std::pair<std::string, geometry_msgs::Pose>>& cube_poses, std::string& current_cube_id, int& current_pick_position, bool& gripper_has_cube, bool execute) {
    if (!gripper_has_cube) {
        ROS_ERROR("CANNOT PLACE A CUBE. GRIPPER IS NOT HOLDING ANY CUBE.");
        return false;
    }

    // Ensure the cube in the gripper is the one specified by action.cube_index1
    if (current_cube_id != cube_poses[action.cube_index1].first) {
        ROS_ERROR("Gripper is holding cube %s, but action specifies cube %d", current_cube_id.c_str(), action.cube_index1);
        return false;
    }

    // Validate the target placement position
    if (action.cube_index2 < 0 || action.cube_index2 >= base_positions.size()) {
        ROS_ERROR("Invalid target position index: %d", action.cube_index2);
        return false;
    }

    // Calculate the place position based on the target location (action.cube_index2)
    geometry_msgs::Pose place_pose = positions.table_center_pose;
    place_pose.position.x += base_positions[action.cube_index2].x;
    place_pose.position.y += base_positions[action.cube_index2].y;

    // Check if there is already a cube at the target position
    std::string target_cube_id;
    if (dynamic_state[action.cube_index2] > 0) {
        // Get the cube already at the target position
        double expected_height = base_positions[action.cube_index2].z + (dynamic_state[action.cube_index2] - 1) * 0.05;

        for (const auto& cube : cube_poses) {
            geometry_msgs::Pose cube_pose = cube.second;
            if (std::abs(cube_pose.position.x - (positions.table_center_pose.position.x + base_positions[action.cube_index2].x)) < 0.01 &&
                std::abs(cube_pose.position.y - (positions.table_center_pose.position.y + base_positions[action.cube_index2].y)) < 0.01 &&
                std::abs(cube_pose.position.z - expected_height) < 0.01) {
                target_cube_id = cube.first;
                break;
            }
        }

        // Allow collision between the current cube and the cube already at the target position
        allowSpecificCubeCollision(current_cube_id, target_cube_id, true);
        ROS_ERROR("Cannot place cube %s at position %d. There are other cubes at this position.", current_cube_id.c_str(), current_pick_position);
        return false;
    }

    // Adjust height based on how many cubes are already present at the target position
    place_pose.position.z = base_positions[action.cube_index2].z + (dynamic_state[action.cube_index2] * 0.05);

    // Log the target place pose
    ROS_INFO("Target place pose: [x: %f, y: %f, z: %f]", place_pose.position.x, place_pose.position.y, place_pose.position.z);

    // Set orientation of the cube being placed
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI / 4);  // Adjust orientation
    place_pose.orientation = tf2::toMsg(q);

    // Attempt to place the cube, return false if it fails
    if (!placeCube(current_cube_id, place_pose, execute)) {
        return false;
    }

    // Update the dynamic state to reflect the new placement
    dynamic_state[action.cube_index2]++;
    cube_poses[action.cube_index1].second = place_pose;

    // Disallow unnecessary collisions after placing
    disallowAllGripperCubeCollisions();
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, cube.first == target_cube_id);
        }
    }

    // Update the cube position after placing
    for (auto& cube : cube_poses) {
        if (cube.first == current_cube_id) {
            cube.second = place_pose;
            break;
        }
    }

    // Reset gripper state after placing
    current_cube_id.clear();
    current_pick_position = -1;
    gripper_has_cube = false;
    dynamic_state[9] = 0;
    if (!execute) {
        planning_scene_interface.clear();
        ros::Duration(0.001).sleep();
        addCollisionObjects(dynamic_state);

    }

    ROS_INFO("PLACE ACTION SUCCEEDED.");
    return true;
}








bool handleStackAction(const Action& action, std::vector<int>& dynamic_state, GoalPositions& positions, std::vector<std::pair<std::string, geometry_msgs::Pose>>& cube_poses, std::string& current_cube_id, int& current_pick_position, bool& gripper_has_cube, bool execute) {
    if (!gripper_has_cube) {
        ROS_ERROR("CANNOT STACK A CUBE. GRIPPER IS NOT HOLDING ANY CUBE.");
        return false;
    }

    // Check if the cube indices are valid.
    if (action.cube_index1 >= cube_poses.size() || action.cube_index2 >= cube_poses.size()) {
        ROS_ERROR("INVALID CUBE INDICES FOR STACK ACTION");
        return false;
    }

    // Ensure the cube in the gripper is the one being stacked.
    if (current_cube_id != cube_poses[action.cube_index1].first) {
        ROS_ERROR("Gripper is holding cube %s, but action specifies cube %d", current_cube_id.c_str(), action.cube_index1);
        return false;
    }

    // Identify the cubes involved in the stack.
    std::string cube_to_move = cube_poses[action.cube_index1].first;
    std::string cube_to_stack_on = cube_poses[action.cube_index2].first;
    geometry_msgs::Pose stack_on_pose = cube_poses[action.cube_index2].second;

    // Determine the position (0 to 8) of the cube to stack on.
    int stack_position = -1;
    for (size_t i = 0; i < base_positions.size(); ++i) {
        if (std::abs(stack_on_pose.position.x - (positions.table_center_pose.position.x + base_positions[i].x)) < 0.01 &&
            std::abs(stack_on_pose.position.y - (positions.table_center_pose.position.y + base_positions[i].y)) < 0.01) {
            stack_position = i;
            break;
        }
    }

    // Check if the stack position was found.
    if (stack_position == -1) {
        ROS_ERROR("COULD NOT FIND THE POSITION OF THE CUBE TO STACK ON");
        return false;
    }

    // --- BEGIN ADDITION: Check if cube_to_stack_on is the top cube at its position ---

    // Determine the exact x and y coordinates of the stack position
    double position_x = positions.table_center_pose.position.x + base_positions[stack_position].x;
    double position_y = positions.table_center_pose.position.y + base_positions[stack_position].y;

    // Collect cubes at the stack position
    std::vector<std::pair<std::string, double>> cubes_at_position;
    for (const auto& cube_pose : cube_poses) {
        const std::string& cube_id = cube_pose.first;
        const geometry_msgs::Pose& pose = cube_pose.second;

        // Check if cube is at the same x and y position
        if (std::abs(pose.position.x - position_x) < 0.01 &&
            std::abs(pose.position.y - position_y) < 0.01) {
            cubes_at_position.push_back(std::make_pair(cube_id, pose.position.z));
        }
    }

    // If no cubes found at position, return error
    if (cubes_at_position.empty()) {
        ROS_ERROR("No cubes found at stack position, but expected at least one (cube to stack on).");
        return false;
    }

    // Sort cubes by z-position (ascending order)
    std::sort(cubes_at_position.begin(), cubes_at_position.end(),
              [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b) {
                  return a.second < b.second;
              });

    // Get the top cube at that position
    const std::string& top_cube_id = cubes_at_position.back().first;

    // Check if the cube to stack on is the top cube
    if (cube_to_stack_on != top_cube_id) {
        ROS_ERROR("Cannot stack onto cube %s because it is not the top cube at its position. The top cube is %s.",
                  cube_to_stack_on.c_str(), top_cube_id.c_str());
        return false;
    }

    // --- END ADDITION ---

    // Adjust the height based on the stack at the current position
    double stack_height = stack_on_pose.position.z + 0.05;  // Adjust height as needed (cube size)

    // Prepare the place pose to stack the cube on top of the target cube.
    geometry_msgs::Pose place_pose = stack_on_pose;
    place_pose.position.z = stack_height;  // Place the cube right on top of the existing stack.

    // Set orientation of the cube being placed
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI/4);
    place_pose.orientation = tf2::toMsg(q);

    // Allow collision between the cube being placed and the cube it is being stacked on.
    allowSpecificCubeCollision(current_cube_id, cube_to_stack_on, true);
    allowGripperCubeCollision(cube_to_stack_on);

    // Place the cube.
    if (!placeCube(current_cube_id, place_pose, execute)) {
        return false;
    }

    // Detach the cube from the gripper after placing it
    detachObject(current_cube_id);
    ROS_INFO("Detached cube %s from gripper after stacking", current_cube_id.c_str());

    // Update the dynamic state to reflect the new stack.
    dynamic_state[stack_position]++;  // Increment the stack height at this position.
    dynamic_state[9] = 0;
    cube_poses[action.cube_index1].second = place_pose;  // Update the pose of the cube that was just placed.

    // Disallow unnecessary collisions after stacking
    disallowAllGripperCubeCollisions();

    // Re-allow collision between the stacked cube and the cube it was placed on (but not with other cubes)
    allowSpecificCubeCollision(cube_to_move, cube_to_stack_on, true);
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id && cube.first != cube_to_stack_on) {
            allowSpecificCubeCollision(current_cube_id, cube.first, false);  // Disallow collision with other cubes
        }
    }

    // Reset gripper state after stacking
    current_cube_id.clear();  // Clear the current cube ID
    gripper_has_cube = false; // The gripper is no longer holding the cube
    current_pick_position = -1;
    if (!execute) {
        planning_scene_interface.clear();
        ros::Duration(0.001).sleep();
        addCollisionObjects(dynamic_state);

    }

    ROS_INFO("STACK ACTION SUCCEEDED.");
    return true;
}




bool handleUnstackAction(const Action& action, std::vector<int>& dynamic_state, GoalPositions& positions, std::vector<std::pair<std::string, geometry_msgs::Pose>>& cube_poses, std::string& current_cube_id, bool& gripper_has_cube, bool execute) {
    // Ensure the gripper is empty before unstacking.
    if (gripper_has_cube) {
        ROS_ERROR("CANNOT UNSTACK A CUBE. GRIPPER IS ALREADY HOLDING A CUBE.");
        return false;
    }

    // Validate the cube indices for unstacking.
    if (action.cube_index1 >= cube_poses.size() || action.cube_index2 >= cube_poses.size()) {
        ROS_ERROR("INVALID CUBE INDICES FOR UNSTACK ACTION");
        return false;
    }

    std::string cube_to_move = cube_poses[action.cube_index1].first;        // The cube we want to unstack.
    std::string cube_to_unstack_from = cube_poses[action.cube_index2].first; // The cube we're unstacking from.
    geometry_msgs::Pose unstack_from_pose = cube_poses[action.cube_index2].second; // Pose of the cube we're unstacking from.
    geometry_msgs::Pose current_cube_pose = cube_poses[action.cube_index1].second;  // Pose of the cube we're unstacking.

    // Ensure the cube to move is directly on top of the cube to unstack from (check Z-coordinate difference).
    double z_diff = current_cube_pose.position.z - unstack_from_pose.position.z;
    if (std::abs(z_diff - 0.05) > 0.01) {  // Allow a small margin of error.
        ROS_ERROR("CANNOT UNSTACK CUBE %s FROM CUBE %s. THEY ARE NOT DIRECTLY STACKED (z-difference: %f).", cube_to_move.c_str(), cube_to_unstack_from.c_str(), z_diff);
        return false;
    }

    // Ensure we are attempting to unstack only if the cubes are correctly aligned on the X and Y coordinates.
    if (std::abs(current_cube_pose.position.x - unstack_from_pose.position.x) > 0.01 ||
        std::abs(current_cube_pose.position.y - unstack_from_pose.position.y) > 0.01) {
        ROS_ERROR("CANNOT UNSTACK CUBE %s FROM CUBE %s. THEY ARE NOT ALIGNED ON X/Y.", cube_to_move.c_str(), cube_to_unstack_from.c_str());
        return false;
    }

    // Find the position of the cube we're unstacking from (cube_index2)
    int unstack_position = -1;
    for (size_t i = 0; i < base_positions.size(); ++i) {
        if (std::abs(unstack_from_pose.position.x - (positions.table_center_pose.position.x + base_positions[i].x)) < 0.01 &&
            std::abs(unstack_from_pose.position.y - (positions.table_center_pose.position.y + base_positions[i].y)) < 0.01) {
            unstack_position = i;
            break;
        }
    }

    // Check if the unstack position was found.
    if (unstack_position == -1) {
        ROS_ERROR("COULD NOT FIND THE POSITION OF THE CUBE TO UNSTACK FROM.");
        return false;
    }

    // Allow gripper and cube collision.
    current_cube_id = cube_to_move;
    allowGripperCubeCollision(current_cube_id);

    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, false);
        }
    }

    // Prepare the gripper to pick up the cube.
    geometry_msgs::Pose pick_pose = current_cube_pose;
    pick_pose.position.z += 0.02;  // Adjust the position to be slightly above the cube for picking.
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI / 4);  // Set the gripper's orientation.
    pick_pose.orientation = tf2::toMsg(q);

    // Attempt to pick the cube.
    if (!pickCube(cube_to_move, pick_pose, execute)) {
        return false;
    }

    // Update the gripper state after picking the cube.
    gripper_has_cube = true;
    dynamic_state[9] = 1;

    // Update the dynamic state to reflect that the cube has been unstacked.
    dynamic_state[unstack_position]--;  // Decrement the count of cubes in the stack.

    // Reset the collision matrix.
    disallowAllGripperCubeCollisions();
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, false);
        }
    }

    if (!execute) {
        planning_scene_interface.clear();
        ros::Duration(0.001).sleep();
        addCollisionObjects(dynamic_state);

    }

    ROS_INFO("UNSTACK ACTION SUCCEEDED. Gripper is now holding cube %s.", current_cube_id.c_str());
    return true;
}






// Get the random number generator
std::mt19937& seedRandomGenerator() {
    static std::random_device rd;  // Seed generator
    static std::mt19937 rng(rd());
    return rng;
}

// Distribute cubes randomly across available positions with valid stacking logic
std::vector<int> distributeCubes(int max_cubes, int max_stack_height, const std::vector<int>& allowed_positions) {
    std::vector<int> dynamic_state(10, 0);  // 9 for grid positions (0-8), 1 for gripper at index 9

    // Get the random number generator
    std::mt19937& rng = seedRandomGenerator();

    // First, randomly decide if the gripper is holding a cube
    std::uniform_int_distribution<int> binary_distribution(0, 1);
    bool gripper_holding = (binary_distribution(rng) == 1);

    int cubes_remaining = max_cubes;

    if (gripper_holding) {
        dynamic_state[9] = 1;  // Gripper is holding a cube
        cubes_remaining -= 1;  // Subtract one cube for the gripper
    }

    int safety_counter = 0;

    // Ensure cubes are placed in valid positions
    std::uniform_int_distribution<int> position_distribution(0, allowed_positions.size() - 1);

    while (cubes_remaining > 0 && safety_counter < 100) {  // Safety counter to prevent infinite loop
        int position_index = position_distribution(rng);
        int position = allowed_positions[position_index];
        int current_height = dynamic_state[position];

        // Only allow stacking if the stack height is within limits
        if (current_height < max_stack_height) {
            dynamic_state[position] += 1;
            cubes_remaining--;
        }

        safety_counter++;
    }

    return dynamic_state;
}

std::vector<int> generateRandomState(int max_cubes, int max_stack_height) {
    // Positions for the 2x2 grid are 0, 1, 3, 4
    std::vector<int> allowed_positions = {0, 1, 3, 4};
    return distributeCubes(max_cubes, max_stack_height, allowed_positions);
}


int getDifferentRandomCube(int num_cubes, int cube_index1) {
    if (num_cubes <= 1) {
        throw std::invalid_argument("Cannot select a different cube when there is only one cube.");
    }

    int cube_index2;
    int safety_counter = 0;
    do {
        cube_index2 = std::rand() % num_cubes;
        safety_counter++;
        if (safety_counter > 100) {
            throw std::runtime_error("Failed to find a different cube. Exiting.");
        }
    } while (cube_index2 == cube_index1);
    return cube_index2;
}


Action generateRandomAction(int num_cubes, int random_action) {
    seedRandomGenerator();

    // Check if num_cubes is valid
    if (num_cubes <= 0) {
        ROS_ERROR("Invalid number of cubes: %d", num_cubes);
        throw std::invalid_argument("Number of cubes must be greater than 0");
    }

    Action::Type random_type = static_cast<Action::Type>(random_action);

    switch (random_type) {
        case Action::PICK: {
            int cube_index = std::rand() % num_cubes;
            return Action(Action::PICK, cube_index);
        }
        case Action::PLACE: {
            int cube_index = std::rand() % num_cubes;
            return Action(Action::PLACE, cube_index, 2);
        }
        case Action::STACK: {
            int cube_index1 = std::rand() % num_cubes;
            int cube_index2 = getDifferentRandomCube(num_cubes, cube_index1);
            return Action(Action::STACK, cube_index1, cube_index2);
        }
        case Action::UNSTACK: {
            int cube_index1 = std::rand() % num_cubes;
            int cube_index2 = getDifferentRandomCube(num_cubes, cube_index1);
            return Action(Action::UNSTACK, cube_index1, cube_index2, 2);
        }
        default:
            ROS_ERROR("Unknown action type generated.");
            throw std::runtime_error("Unknown action type generated.");
    }
}


  struct CubePosition {
        double x, y, z;
        CubePosition(double x, double y, double z) : x(x), y(y), z(z) {}
    };

    // Method to add collision objects based on a given state
    void addCollisionObjects(const State& state)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Get the current pose of the end-effector
    geometry_msgs::PoseStamped current_pose = arm_move_group.getCurrentPose();

    // Calculate goal pose 1 (table position)
    geometry_msgs::Pose goal_pose1 = current_pose.pose;
    goal_pose1.position.x += 0.225;  // 22.5cm forward

    geometry_msgs::Pose goal_pose2 = current_pose.pose;
    goal_pose2.position.y += 0.3;  // 30cm to the right

    // Create Table 1
    moveit_msgs::CollisionObject table1;
    table1.header.frame_id = "panda_link0";
    table1.id = "table1";
    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions = {0.4, 0.4, 0.4};  // 40x40x40 cm
    geometry_msgs::Pose table1_pose;
    table1_pose.orientation.w = 1.0;
    table1_pose.position = goal_pose1.position;
    table1_pose.position.z = 0.2;  // Half of table height
    table1.primitives.push_back(primitive1);
    table1.primitive_poses.push_back(table1_pose);
    table1.operation = table1.ADD;
    collision_objects.push_back(table1);

    // // Create Table 2
    // moveit_msgs::CollisionObject table2;
    // table2.header.frame_id = "panda_link0";
    // table2.id = "table2";
    // shape_msgs::SolidPrimitive primitive2;
    // primitive2.type = primitive2.BOX;
    // primitive2.dimensions = {0.25, 0.175, 0.4};  // 25x17.5x40 cm
    // geometry_msgs::Pose table2_pose;
    // table2_pose.orientation.w = 1.0;
    // table2_pose.position.x = goal_pose2.position.x;
    // table2_pose.position.y = goal_pose2.position.y;
    // table2_pose.position.z = 0.2;  // Half of table height
    // table2.primitives.push_back(primitive2);
    // table2.primitive_poses.push_back(table2_pose);
    // table2.operation = table2.ADD;
    // collision_objects.push_back(table2);

    // Define the base positions for the 3x3 grid
    std::vector<CubePosition> base_positions = {
        CubePosition(-0.15, -0.15, 0.425),
        CubePosition(-0.15, 0, 0.425),
        CubePosition(-0.15, 0.15, 0.425),
        CubePosition(0, -0.15, 0.425),
        CubePosition(0, 0, 0.425),
        CubePosition(0, 0.15, 0.425),
        CubePosition(0.15, -0.15, 0.425),
        CubePosition(0.15, 0, 0.425),
        CubePosition(0.15, 0.15, 0.425)
    };

    int cube_count = 0;
    bool gripper_has_cube = state[9] == 1;

    // Add cube for gripper if it's holding one
    if (gripper_has_cube) {
        moveit_msgs::CollisionObject gripper_cube;
        gripper_cube.header.frame_id = "panda_link0";
        gripper_cube.id = "small_cube0";
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.05, 0.05, 0.05};  // 5x5x5 cm cube

        geometry_msgs::Pose cube_pose;
        cube_pose.orientation.w = 1.0;
        cube_pose.position.x = 0.306;
        cube_pose.position.y = 0;
        cube_pose.position.z = 0.47;

        gripper_cube.primitives.push_back(primitive);
        gripper_cube.primitive_poses.push_back(cube_pose);
        gripper_cube.operation = gripper_cube.ADD;
        collision_objects.push_back(gripper_cube);

        cube_count++;
    }

    // Create cubes based on the state
    for (size_t i = 0; i < state.size() - 1; ++i) {  // Ignore the last element (gripper state)
        for (int j = 0; j < state[i]; ++j) {
            moveit_msgs::CollisionObject small_cube;
            small_cube.header.frame_id = "panda_link0";
            small_cube.id = "small_cube" + std::to_string(cube_count++);
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {0.05, 0.05, 0.05};  // 5x5x5 cm cube

            geometry_msgs::Pose cube_pose;
            cube_pose.orientation.w = 1.0;
            cube_pose.position.x = goal_pose1.position.x + base_positions[i].x;
            cube_pose.position.y = goal_pose1.position.y + base_positions[i].y;
            cube_pose.position.z = base_positions[i].z + j * 0.05;  // Stack cubes

            small_cube.primitives.push_back(primitive);
            small_cube.primitive_poses.push_back(cube_pose);
            small_cube.operation = small_cube.ADD;
            collision_objects.push_back(small_cube);
        }
    }

    // Add all collision objects to the scene
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Added tables and %d small cubes (including gripper cube if present) as collision objects", cube_count);
}


// Function to convert an Action to a string representation
std::string actionToString(const Action& action) {
    std::stringstream ss;
    switch (action.type) {
        case Action::PICK:
            ss << "PICK," << action.cube_index1 << ",9";
            break;
        case Action::PLACE:
            ss << "PLACE," << action.cube_index1 << ",9";
            break;
        case Action::STACK:
            ss << "STACK," << action.cube_index1 << "," << action.cube_index2;
            break;
        case Action::UNSTACK:
            ss << "UNSTACK," << action.cube_index1 << "," << action.cube_index2;
            break;
    }
    return ss.str();
}

// Function to convert a State and Action to a unique string representation
std::string stateActionToString(const State& state, const Action& action) {
    std::stringstream ss;
    for (int val : state) {
        ss << val << ",";
    }
    ss << actionToString(action);
    return ss.str();
}

std::tuple<std::vector<State>, std::vector<Action>, std::vector<bool>, std::vector<State>>
runSimulations(int num_simulations, int max_cubes, int max_height, bool execute,
                  bool action_select, int action_choice) {
    std::vector<State> states;
    std::vector<Action> actions;
    std::vector<bool> results;
    std::vector<State> resulting_states;

    std::set<std::string> unique_plans;

    int simulations_completed = 0;
    int attempts = 0;
    int max_attempts = num_simulations * 5;  // To prevent infinite loops

    while (simulations_completed < num_simulations && attempts < max_attempts) {
        attempts++;
        try {
            ROS_INFO("Running simulation %d...", simulations_completed + 1);

            State state = generateRandomState(max_cubes, max_height);
            // Use action_choice if action_select is true; otherwise, generate a random action
            int action_type;
            if (action_select) {
                action_type = action_choice;
            } else {
                action_type = std::rand() % 4;  // Assuming there are 4 action types (0 to 3)
            }
            Action action = generateRandomAction(max_cubes, action_type);

            // Validate action indices
            if (action.cube_index1 >= max_cubes || (action.type == Action::STACK && action.cube_index2 >= max_cubes) || (action.type == Action::UNSTACK && action.cube_index2 >= max_cubes)) {
                ROS_ERROR("Generated an invalid action. Skipping this simulation.");
                continue; // Skip this simulation
            }

            // Create a string representation of the state-action pair
            std::string plan_key = stateActionToString(state, action);

            // Check for uniqueness
            if (unique_plans.find(plan_key) != unique_plans.end()) {
                ROS_INFO("Duplicate Plan Found, Skipping");
                continue;
            }

            // Add to unique plans
            unique_plans.insert(plan_key);

            // Initialize the planner's dynamic_state with the initial state
            dynamic_state = state;

            // Run the simulation with the generated state and action
            bool result = run({action}, state, execute);

            // Store results
            states.push_back(state);
            actions.push_back(action);
            results.push_back(result);
            resulting_states.push_back(dynamic_state);

            simulations_completed++;

            ROS_INFO("Completed simulation %d of %d", simulations_completed, num_simulations);
            ROS_INFO("Unique plans generated so far: %d", simulations_completed);

        } catch (const std::exception &e) {
            ROS_ERROR("Exception during simulation: %s", e.what());
            continue;
        } catch (...) {
            ROS_ERROR("Unknown error during simulation");
            continue;
        }

        // Optional: Pause between simulations
        if (simulations_completed < num_simulations) {
            ROS_INFO("Waiting 1 second before starting the next simulation...");
            ros::Duration(1.0).sleep();
        }
    }

    if (attempts >= max_attempts) {
        ROS_WARN("Reached maximum number of attempts. Completed %d unique simulations.", simulations_completed);
    }

    ROS_INFO("All simulations complete. Total unique simulations: %d", simulations_completed);
    return {states, actions, results, resulting_states};
}


struct PDDLAction {
    std::string name;
    std::set<std::string> preconditions;
    std::set<std::string> effects;
};





std::set<std::string> convertStateToPDDLPredicatesInitial(const State& state, int num_objects) {
    bool gripper_holding = state[9] == 1;
    std::set<std::string> predicates;
    std::vector<std::vector<std::string>> blocks_on_table(state.size() - 1);

    // Adjust the object IDs based on whether the gripper is holding a cube
    std::vector<std::string> objects;
    int object_index = gripper_holding ? 1 : 0;
    for (int i = 0; i < num_objects; ++i) {
        objects.push_back("cube_" + std::to_string(object_index));
        ++object_index;
    }
    if (gripper_holding) {
        objects.insert(objects.begin(), "cube_0");  // Add cube_0 for the held cube
    }

    // Initialize block positions
    int block_index = gripper_holding ? 1 : 0;
    for (size_t i = 0; i < state.size() - 1; ++i) {
        int stack_height = state[i];
        for (int j = 0; j < stack_height; ++j) {
            blocks_on_table[i].push_back(objects[block_index]);
            ++block_index;
        }
    }

    // Create PDDL predicates for blocks on table and stacked blocks
    for (size_t i = 0; i < blocks_on_table.size(); ++i) {
        if (!blocks_on_table[i].empty()) {
            predicates.insert(blocks_on_table[i][0] + "onTable");
            if (blocks_on_table[i].size() > 1) {
                for (size_t j = 1; j < blocks_on_table[i].size(); ++j) {
                    predicates.insert(blocks_on_table[i][j] + "on" + blocks_on_table[i][j - 1]);
                }
                predicates.insert(blocks_on_table[i].back() + "isTopMost");
            } else {
                predicates.insert(blocks_on_table[i][0] + "isTopMost");
            }
        }
    }

    // Handle gripper state
    if (gripper_holding) {
        predicates.insert("holdingcube_0");
    } else {
        predicates.insert("gripperFree");
    }

    return predicates;
}

std::set<std::string> convertStateToPDDLPredicatesFinal(const State& initial_state, const State& final_state, int num_objects) {
    std::set<std::string> predicates;
    std::vector<std::vector<std::string>> initial_detailed_state(10);
    std::vector<std::vector<std::string>> final_detailed_state(10);

    // Initialize the detailed state arrays
    int cube_id = initial_state[9] == 1 ? 1 : 0;  // Start from 1 if gripper is holding, else from 0
    
    // Handle gripper first if it's holding a cube
    if (initial_state[9] == 1) {
        initial_detailed_state[9].push_back("cube_0");
    }

    // Initialize cubes on the table
    for (size_t i = 0; i < 9; ++i) {
        for (int j = 0; j < initial_state[i]; ++j) {
            initial_detailed_state[i].push_back("cube_" + std::to_string(cube_id++));
        }
    }

    // Copy initial detailed state to final detailed state
    final_detailed_state = initial_detailed_state;

    // Handle the action (pick/unstack or place/stack)
    if (initial_state[9] == 0 && final_state[9] == 1) {
        // Pick or Unstack action
        for (size_t i = 0; i < 9; ++i) {
            if (final_state[i] < initial_state[i]) {
                final_detailed_state[9].push_back(final_detailed_state[i].back());
                final_detailed_state[i].pop_back();
                break;
            }
        }
    } else if (initial_state[9] == 1 && final_state[9] == 0) {
        // Place or Stack action
        for (size_t i = 0; i < 9; ++i) {
            if (final_state[i] > initial_state[i]) {
                final_detailed_state[i].push_back(final_detailed_state[9].back());
                final_detailed_state[9].clear();
                break;
            }
        }
    }

    // Create PDDL predicates for the final state
    for (size_t i = 0; i < 9; ++i) {
        if (!final_detailed_state[i].empty()) {
            predicates.insert(final_detailed_state[i][0] + "onTable");
            if (final_detailed_state[i].size() > 1) {
                for (size_t j = 1; j < final_detailed_state[i].size(); ++j) {
                    predicates.insert(final_detailed_state[i][j] + "on" + final_detailed_state[i][j-1]);
                }
            }
            predicates.insert(final_detailed_state[i].back() + "isTopMost");
        }
    }

    // Handle gripper state
    if (final_state[9] == 1) {
        predicates.insert("holding" + final_detailed_state[9][0]);
    } else {
        predicates.insert("gripperFree");
    }

    return predicates;
}


// Scrambled version
PDDLAction convertActionToPDDLAction(const Action& action) {
    PDDLAction pddl_action;
    switch (action.type) {
        case Action::PICK: {
            std::string cube_id = "cube_" + std::to_string(action.cube_index1);
            pddl_action.name = "pick-" + cube_id;
            pddl_action.preconditions = {"gripperFree", cube_id + "isTopMost"}; // Removed onTable condition
            pddl_action.effects = {"holding" + cube_id, "not gripperFree", cube_id + "onTable"}; // Added onTable effect instead of removing it
            break;
        }
        case Action::PLACE: {
            std::string cube_id = "cube_" + std::to_string(action.cube_index1);
            pddl_action.name = "place-" + cube_id;
            pddl_action.preconditions = {"holding" + cube_id, cube_id + "isTopMost"}; // Added isTopMost condition
            pddl_action.effects = {"not holding" + cube_id, "gripperFree", "not " + cube_id + "onTable"}; // Removed onTable effect
            break;
        }
        case Action::STACK: {
            std::string cube_id_a = "cube_" + std::to_string(action.cube_index1);
            std::string cube_id_b = "cube_" + std::to_string(action.cube_index2);
            pddl_action.name = "stack-" + cube_id_a + "-onto-" + cube_id_b;
            pddl_action.preconditions = {"holding" + cube_id_a, cube_id_b + "onTable"}; // Changed isTopMost to onTable for cube_id_b
            pddl_action.effects = {"not holding" + cube_id_a, cube_id_a + "on" + cube_id_b, "not " + cube_id_a + "isTopMost"}; // Removed gripperFree effect, changed isTopMost effects
            break;
        }
        case Action::UNSTACK: {
            std::string cube_id_a = "cube_" + std::to_string(action.cube_index1);
            std::string cube_id_b = "cube_" + std::to_string(action.cube_index2);
            pddl_action.name = "unstack-" + cube_id_a + "-from-" + cube_id_b;
            pddl_action.preconditions = {cube_id_a + "on" + cube_id_b, "gripperFree"}; // Removed isTopMost condition for cube_id_a
            pddl_action.effects = {"holding" + cube_id_a, "not " + cube_id_a + "on" + cube_id_b, cube_id_a + "isTopMost"}; // Added isTopMost effect for cube_id_a, removed for cube_id_b
            break;
        }
        default:
            throw std::runtime_error("Unknown action type");
    }
    return pddl_action;
}


// // Original correct version
// PDDLAction convertActionToPDDLAction(const Action& action) {
//     PDDLAction pddl_action;
//     switch (action.type) {
//         case Action::PICK: {
//             std::string cube_id = "cube_" + std::to_string(action.cube_index1);
//             pddl_action.name = "pick-" + cube_id;
//             pddl_action.preconditions = {"gripperFree", cube_id + "onTable", cube_id + "isTopMost"};
//             pddl_action.effects = {"holding" + cube_id, "not gripperFree", "not " + cube_id + "onTable", "not " + cube_id + "isTopMost"};
//             break;
//         }
//         case Action::PLACE: {
//             std::string cube_id = "cube_" + std::to_string(action.cube_index1);
//             pddl_action.name = "place-" + cube_id;
//             pddl_action.preconditions = {"holding" + cube_id};
//             pddl_action.effects = {"not holding" + cube_id, "gripperFree", cube_id + "onTable", cube_id + "isTopMost"};
//             break;
//         }
//         case Action::STACK: {
//             std::string cube_id_a = "cube_" + std::to_string(action.cube_index1);
//             std::string cube_id_b = "cube_" + std::to_string(action.cube_index2);
//             pddl_action.name = "stack-" + cube_id_a + "-onto-" + cube_id_b;
//             pddl_action.preconditions = {"holding" + cube_id_a, cube_id_b + "isTopMost"};
//             pddl_action.effects = {"not holding" + cube_id_a, "gripperFree", cube_id_a + "on" + cube_id_b, cube_id_a + "isTopMost", "not " + cube_id_b + "isTopMost"};
//             break;
//         }
//         case Action::UNSTACK: {
//             std::string cube_id_a = "cube_" + std::to_string(action.cube_index1);
//             std::string cube_id_b = "cube_" + std::to_string(action.cube_index2);
//             pddl_action.name = "unstack-" + cube_id_a + "-from-" + cube_id_b;
//             pddl_action.preconditions = {cube_id_a + "on" + cube_id_b, cube_id_a + "isTopMost", "gripperFree"};
//             pddl_action.effects = {"holding" + cube_id_a, "not gripperFree", "not " + cube_id_a + "on" + cube_id_b, cube_id_b + "isTopMost"};
//             break;
//         }
//         default:
//             throw std::runtime_error("Unknown action type");
//     }
//     return pddl_action;
// }


bool canApplyAction(const std::set<std::string>& state_predicates, const PDDLAction& pddl_action) {
    for (const auto& precondition : pddl_action.preconditions) {
        if (precondition.find("not ") == 0) {
            // Negative precondition
            std::string positive_predicate = precondition.substr(4);
            if (state_predicates.find(positive_predicate) != state_predicates.end()) {
                // Positive predicate exists in state, so negative precondition fails
                return false;
            }
        } else {
            // Positive precondition
            if (state_predicates.find(precondition) == state_predicates.end()) {
                // Precondition not found in state
                return false;
            }
        }
    }
    return true;
}

bool checkEffects(const std::set<std::string>& state_predicates, const std::set<std::string>& effects) {
    for (const auto& effect : effects) {
        if (effect.find("not ") == 0) {
            // Negative effect: the predicate should not be present in the state
            std::string positive_predicate = effect.substr(4);
            if (state_predicates.find(positive_predicate) != state_predicates.end()) {
                // Positive predicate exists in state, so negative effect is not satisfied
                return false;
            }
        } else {
            // Positive effect: the predicate should be present in the state
            if (state_predicates.find(effect) == state_predicates.end()) {
                // Predicate not found in state, so positive effect is not satisfied
                return false;
            }
        }
    }
    return true;
}



void runTaskPlanning(const std::vector<State>& states,
                     const std::vector<Action>& actions,
                     const std::vector<bool>& motion_results,
                     const std::vector<State>& resulting_states,
                     std::vector<bool>& precondition_results,
                     std::vector<bool>& effect_results) {
    precondition_results.resize(states.size(), false);
    effect_results.resize(states.size(), false);
    for (size_t i = 0; i < states.size(); ++i) {
        const State& initial_state = states[i];
        const Action& action = actions[i];
        bool motion_result = motion_results[i];
        const State& resulting_state = resulting_states[i];

        // Count objects in initial state
        int num_objects = 0;
        for (size_t j = 0; j < initial_state.size() - 1; ++j) {
            num_objects += initial_state[j];
        }
        if (initial_state[9] == 1) {
            num_objects += 1;  // Include cube held by gripper
        }

        // Convert the initial state to PDDL predicates
        std::set<std::string> initial_state_predicates = convertStateToPDDLPredicatesInitial(initial_state, num_objects);
        PDDLAction pddl_action = convertActionToPDDLAction(action);

        // Check if the action's preconditions are satisfied
        bool preconditions_satisfied = canApplyAction(initial_state_predicates, pddl_action);
        precondition_results[i] = preconditions_satisfied;

        // Initialize effect result to false
        effect_results[i] = false;

        if (!preconditions_satisfied) {
            // Precondition not satisfied; cannot proceed to effect checking
            continue;
        }

        if (!motion_result) {
            // Motion planning failed; effect checking is not applicable
            continue;
        }

        // Convert the resulting state to PDDL predicates
        int num_objects_resulting = 0;
        for (size_t j = 0; j < resulting_state.size() - 1; ++j) {
            num_objects_resulting += resulting_state[j];
        }
        if (resulting_state[9] == 1) {
            num_objects_resulting += 1;  // Include cube held by gripper
        }

        // Convert the resulting state to PDDL predicates
        std::set<std::string> resulting_state_predicates = convertStateToPDDLPredicatesFinal(initial_state, resulting_state, num_objects);

        // Check if the effects are correctly reflected in the resulting state
        bool effects_satisfied = checkEffects(resulting_state_predicates, pddl_action.effects);
        effect_results[i] = effects_satisfied;
    }
}




struct CubeInfo {
    std::string cube_id;        // e.g., "cube_0"
    int position_index;         // 0-8 for grid positions, -1 if held by gripper
    int stack_level;            // 0 for bottom cube, increasing upwards
    bool is_held_by_gripper;    // True if the cube is held by the gripper
};



std::vector<CubeInfo> constructCubeInfo(const State& state) {
    std::vector<CubeInfo> cubes;
    bool gripper_holding = state[9] == 1;

    // Calculate total number of cubes
    int num_cubes = 0;
    for (size_t i = 0; i < state.size() - 1; ++i) {
        num_cubes += state[i];
    }
    if (gripper_holding) {
        num_cubes += 1;
    }

    // Generate cube IDs consistent with task planning
    std::vector<std::string> cube_ids;
    int object_index = gripper_holding ? 1 : 0;
    for (int i = 0; i < num_cubes - gripper_holding; ++i) {
        cube_ids.push_back("cube_" + std::to_string(object_index));
        ++object_index;
    }
    if (gripper_holding) {
        cube_ids.insert(cube_ids.begin(), "cube_0");  // The cube held by the gripper is cube_0
    }

    int cube_index = gripper_holding ? 1 : 0;  // Start index for cubes in positions

    // Process grid positions
    for (int position = 0; position <= 8; ++position) {
        int stack_height = state[position];
        for (int level = 0; level < stack_height; ++level) {
            CubeInfo cube;
            cube.cube_id = cube_ids[cube_index++];
            cube.position_index = position;
            cube.stack_level = level;
            cube.is_held_by_gripper = false;
            cubes.push_back(cube);
        }
    }

    // Handle cube held by gripper
    if (gripper_holding) {
        CubeInfo cube;
        cube.cube_id = "cube_0";
        cube.position_index = -1;  // Indicates held by gripper
        cube.stack_level = -1;
        cube.is_held_by_gripper = true;
        cubes.push_back(cube);
    }

    return cubes;
}


void printCubeInfo(const std::vector<CubeInfo>& cubes) {
    // Group cubes by position
    std::map<int, std::vector<CubeInfo>> position_to_cubes;
    CubeInfo gripper_cube;
    bool gripper_holding = false;

    for (const auto& cube : cubes) {
        if (cube.is_held_by_gripper) {
            gripper_cube = cube;
            gripper_holding = true;
        } else {
            position_to_cubes[cube.position_index].push_back(cube);
        }
    }

    // Sort cubes at each position by stack level
    for (auto& entry : position_to_cubes) {
        auto& cube_list = entry.second;
        std::sort(cube_list.begin(), cube_list.end(), [](const CubeInfo& a, const CubeInfo& b) {
            return a.stack_level < b.stack_level;
        });
    }

    // Print cube positions and relationships
    std::cout << "Cube Positions and Relationships:\n";
    for (const auto& entry : position_to_cubes) {
        int position = entry.first;
        const auto& cube_list = entry.second;
        std::cout << "Position " << position << ":\n";
        for (size_t i = 0; i < cube_list.size(); ++i) {
            const auto& cube = cube_list[i];
            std::cout << "  " << cube.cube_id;
            if (i == 0) {
                std::cout << " is on the table";
            } else {
                std::cout << " is on top of " << cube_list[i - 1].cube_id;
            }
            std::cout << "\n";
        }
    }

    // Print gripper status
    if (gripper_holding) {
        std::cout << "Gripper is holding " << gripper_cube.cube_id << "\n";
    } else {
        std::cout << "Gripper is empty\n";
    }
}




void printResults(const std::vector<State>& states, const std::vector<Action>& actions,
                  const std::vector<bool>& motion_results, const std::vector<bool>& precondition_results,
                  const std::vector<bool>& effect_results, const std::vector<State>& resulting_states) {
    std::cout << "Simulation Results:\n";
    std::cout << "--------------------------------------------\n";
    for (size_t i = 0; i < states.size(); ++i) {
        printSimulationResult(i, states[i], actions[i], motion_results[i], precondition_results[i], effect_results[i], resulting_states[i]);
    }
}

void printSimulationResult(size_t index, const State& state, const Action& action,
                           bool motion_result, bool precondition_result, bool effect_result, const State& resulting_state) {
    std::cout << "Simulation " << index + 1 << ":\n";
    printState("Initial State", state);
    printAction(action);
    std::cout << "  Motion Planning Result: " << (motion_result ? "SUCCESS" : "FAILURE") << "\n";
    std::cout << "  Precondition Check: " << (precondition_result ? "PASSED" : "FAILED") << "\n";
    if (motion_result && precondition_result) {
        std::cout << "  Effect Check: " << (effect_result ? "PASSED" : "FAILED") << "\n";
    } else {
        std::cout << "  Effect Check: N/A\n";
    }
    printState("Resulting State", resulting_state);
    std::cout << "--------------------------------------------\n";
}



void printState(const std::string& label, const State& state) {
    std::cout << "  " << label << ": [ ";
    for (size_t j = 0; j < 9; ++j) {
        std::cout << state[j] << " ";
    }
    std::cout << "] Gripper: " << (state[9] == 1 ? "Holding cube" : "Empty") << "\n";
}


void printAction(const Action& action) {
    std::cout << "  Action: ";
    switch (action.type) {
        case Action::PICK:
            std::cout << "PICK Cube Index: " << action.cube_index1;
            break;
        case Action::PLACE:
            std::cout << "PLACE Cube Index: " << action.cube_index1 << " at Position: " << action.position_index;
            break;
        case Action::STACK:
            std::cout << "STACK Cube " << action.cube_index1 << " on Cube " << action.cube_index2;
            break;
        case Action::UNSTACK:
            std::cout << "UNSTACK Cube " << action.cube_index1 << " from Cube " << action.cube_index2;
            break;
    }
    std::cout << "\n";
}

void printTaskState(const std::set<std::string>& state_predicates) {
    for (const auto& predicate : state_predicates) {
        std::cout << "  " << predicate << "\n";
    }
}

void printTaskAction(const PDDLAction& pddl_action) {
    std::cout << "  Action Name: " << pddl_action.name << "\n";
    std::cout << "  Preconditions:\n";
    for (const auto& precondition : pddl_action.preconditions) {
        std::cout << "    " << precondition << "\n";
    }
    std::cout << "  Effects:\n";
    for (const auto& effect : pddl_action.effects) {
        std::cout << "    " << effect << "\n";
    }
}


void printSimpleResults(const std::vector<State>& states, const std::vector<Action>& actions,
                        const std::vector<bool>& motion_results, const std::vector<bool>& precondition_results,
                        const std::vector<bool>& effect_results, const std::vector<State>& resulting_states) {
    // Print the states
    std::cout << "States:\n[";
    for (size_t i = 0; i < states.size(); ++i) {
        printStateArray(states[i]);
        if (i < states.size() - 1) std::cout << ",\n "; // Add newline and indent for better readability
    }
    std::cout << "]\n\n";

    // Print the actions
    std::cout << "Actions:\n[";
    for (size_t i = 0; i < actions.size(); ++i) {
        printActionArray(actions[i]);
        if (i < actions.size() - 1) std::cout << ",\n "; // Add newline and indent for better readability
    }
    std::cout << "]\n\n";

    // Print the results side by side
    std::cout << "Results:\n";
    std::cout << "Index\tMotion Planning\tPreconditions\tEffects\n";
    std::cout << "-----------------------------------------------------------\n";
    for (size_t i = 0; i < motion_results.size(); ++i) {
        std::cout << i << "\t";
        std::cout << (motion_results[i] ? "SUCCESS" : "FAILURE") << "\t\t";
        std::cout << (precondition_results[i] ? "PASSED" : "FAILED") << "\t\t";
        if (motion_results[i] && precondition_results[i]) {
            std::cout << (effect_results[i] ? "PASSED" : "FAILED") << "\n";
        } else {
            std::cout << "N/A\n";
        }
    }

    // Print the resulting states
    std::cout << "\nResulting States:\n[";
    for (size_t i = 0; i < resulting_states.size(); ++i) {
        printStateArray(resulting_states[i]);
        if (i < resulting_states.size() - 1) std::cout << ",\n "; // Add newline and indent for better readability
    }
    std::cout << "]\n\n";
}

void printStateArray(const State& state) {
    std::cout << "[";
    for (size_t j = 0; j < 9; ++j) {
        std::cout << state[j] << ", ";
    }
    std::cout << "\"Gripper: " << (state[9] == 1 ? "Holding cube" : "Empty") << "\"]";
}

void printActionArray(const Action& action) {
    std::cout << "[\"";
    switch (action.type) {
        case Action::PICK:
            std::cout << "PICK\", " << action.cube_index1;
            break;
        case Action::PLACE:
            std::cout << "PLACE\", " << action.cube_index1 << ", " << action.cube_index2;
            break;
        case Action::STACK:
            std::cout << "STACK\", " << action.cube_index1 << ", " << action.cube_index2;
            break;
        case Action::UNSTACK:
            std::cout << "UNSTACK\", " << action.cube_index1 << ", " << action.cube_index2;
            break;
    }
    std::cout << "]";
}


void printDiscrepancies(const std::vector<std::tuple<State, Action, std::string>>& discrepancies_preconditions,
                        const std::vector<std::tuple<State, Action, State, std::string>>& discrepancies_effects) {
    if (discrepancies_preconditions.empty()) {
    std::cout << "No discrepancies due to precondition failures.\n";
    } else {
        std::cout << "\nDiscrepancies Due to Precondition Failures:\n";
        for (size_t i = 0; i < discrepancies_preconditions.size(); ++i) {
            const State& state = std::get<0>(discrepancies_preconditions[i]);
            const Action& action = std::get<1>(discrepancies_preconditions[i]);
            const std::string& info = std::get<2>(discrepancies_preconditions[i]);

            // Convert action to PDDL action to get preconditions
            PDDLAction pddl_action = convertActionToPDDLAction(action);

            std::cout << "Discrepancy " << i + 1 << ":\n";
            printState("Motion Planning State", state);
            printAction(action);

            // Construct and print cube information
            std::vector<CubeInfo> cubes = constructCubeInfo(state);
            printCubeInfo(cubes);

            // Print PDDL action preconditions
            std::cout << "PDDL Action Preconditions:\n";
            for (const auto& precondition : pddl_action.preconditions) {
                std::cout << "  " << precondition << "\n";
            }

            std::cout << "  " << info << "\n";
            std::cout << "--------------------------------------------\n";
        }
        std::cout << "Total discrepancies due to preconditions: " << discrepancies_preconditions.size() << "\n";
    }

    if (discrepancies_effects.empty()) {
        std::cout << "No discrepancies due to effect mismatches.\n";
    } else {
        std::cout << "\nDiscrepancies Due to Effect Mismatches:\n";
        for (size_t i = 0; i < discrepancies_effects.size(); ++i) {
            const State& initial_state = std::get<0>(discrepancies_effects[i]);
            const Action& action = std::get<1>(discrepancies_effects[i]);
            const State& resulting_state = std::get<2>(discrepancies_effects[i]);
            const std::string& info = std::get<3>(discrepancies_effects[i]);

            // Convert action to PDDL action to get expected effects
            PDDLAction pddl_action = convertActionToPDDLAction(action);

            std::cout << "Discrepancy " << i + 1 << ":\n";
            std::cout << "Before Action:\n";
            printState("Initial Motion Planning State", initial_state);
            printAction(action);

            // Print expected effects
            std::cout << "Expected Effects:\n";
            for (const auto& effect : pddl_action.effects) {
                std::cout << "  " << effect << "\n";
            }

            // Print resulting state
            std::cout << "After Action:\n";
            printState("Resulting Motion Planning State", resulting_state);

            // Construct and print cube information for resulting state
            std::vector<CubeInfo> resulting_cubes = constructCubeInfo(resulting_state);
            printCubeInfo(resulting_cubes);

            // Convert resulting state to PDDL predicates
            int num_objects = 0;
            for (size_t j = 0; j < resulting_state.size() - 1; ++j) {
                num_objects += resulting_state[j];
            }
            if (resulting_state[9] == 1) {
                num_objects += 1;  // Include cube held by gripper
            }
            std::set<std::string> resulting_state_predicates = convertStateToPDDLPredicatesFinal(initial_state, resulting_state, num_objects);

            // Print resulting state predicates
            std::cout << "Resulting State Predicates:\n";
            for (const auto& predicate : resulting_state_predicates) {
                std::cout << "  " << predicate << "\n";
            }

            // Compare expected effects with resulting state predicates
            std::cout << "Effects Not Satisfied:\n";
            bool all_effects_satisfied = true;
            for (const auto& effect : pddl_action.effects) {
                bool effect_satisfied;
                if (effect.find("not ") == 0) {
                    // Negative effect: predicate should not be in resulting state
                    std::string positive_predicate = effect.substr(4);
                    effect_satisfied = (resulting_state_predicates.find(positive_predicate) == resulting_state_predicates.end());
                } else {
                    // Positive effect: predicate should be in resulting state
                    effect_satisfied = (resulting_state_predicates.find(effect) != resulting_state_predicates.end());
                }
                if (!effect_satisfied) {
                    all_effects_satisfied = false;
                    std::cout << "  " << effect << "\n";
                }
            }

            if (all_effects_satisfied) {
                std::cout << "  All effects are satisfied (unexpected since there's a discrepancy)\n";
            }

            std::cout << "  " << info << "\n";
            std::cout << "--------------------------------------------\n";
        }
        std::cout << "Total discrepancies due to effects: " << discrepancies_effects.size() << "\n";
    }
}



void printMetrics(int total_plans, int motion_successes, int motion_failures,
                  int precondition_successes, int precondition_failures,
                  int effect_successes, int effect_failures,
                  int black_list_type_1, int white_list_type_1,
                  int black_list_type_2, int white_list_type_2) {
    std::cout << "\n--- Metrics Summary ---\n";
    std::cout << "Total Plans Generated: " << total_plans << "\n";
    std::cout << "Motion Planning Successes: " << motion_successes << "\n";
    std::cout << "Motion Planning Failures: " << motion_failures << "\n";
    std::cout << "Precondition Successes: " << precondition_successes << "\n";
    std::cout << "Precondition Failures: " << precondition_failures << "\n";
    std::cout << "Effect Successes: " << effect_successes << "\n";
    std::cout << "Effect Failures: " << effect_failures << "\n\n";

    std::cout << "Classification Counts (Based on Motion Planning and Precondition Results):\n";
    std::cout << "Black List Type 1 (Both Failed): " << black_list_type_1 << "\n";
    std::cout << "White List Type 1 (Both Succeeded): " << white_list_type_1 << "\n";
    std::cout << "Black List Type 2 (Precondition Succeeded, Motion Failed): " << black_list_type_2 << "\n";
    std::cout << "White List Type 2 (Precondition Failed, Motion Succeeded): " << white_list_type_2 << "\n";
}





private:
  moveit::planning_interface::MoveGroupInterface arm_move_group;
  moveit::planning_interface::MoveGroupInterface gripper_move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit_visual_tools::MoveItVisualTools visual_tools;
  Eigen::Isometry3d text_pose;
  moveit_msgs::OrientationConstraint ocm;
  moveit_msgs::Constraints test_constraints;
  std::vector<CubePosition> base_positions;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_motion_planner");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    // Instantiate the planner
    PandaMotionPlanner planner;

    // Whether we want to run a simulation or manually test
    bool simulation_or_test = false;

    // Whether we want to see actions executed or not
    bool execute = false;


    // Running a simulation
    if (simulation_or_test) {
    // Set the number of unique simulations to run
    int num_simulations = 50; // Desired number of unique simulations
    int max_cubes = 2;
    int max_height = 2;

    // Whether we want to manually select actions or not
    bool action_select = true;
    bool action_choice = 0;


    // Start the timer
    auto start_time = std::chrono::high_resolution_clock::now();

    // Run the simulations and get the results
    auto [states, actions, results, resulting_states] = planner.runSimulations(num_simulations, max_cubes, max_height, execute, action_select, action_choice);

    // Initialize vectors for precondition and effect results
    std::vector<bool> precondition_results;
    std::vector<bool> effect_results;

    // Run the task planning and get the precondition and effect results
    planner.runTaskPlanning(states, actions, results, resulting_states, precondition_results, effect_results);

    // Collect discrepancies and compute metrics
    std::vector<std::tuple<PandaMotionPlanner::State, PandaMotionPlanner::Action, std::string>> discrepancies_preconditions;
    std::vector<std::tuple<PandaMotionPlanner::State, PandaMotionPlanner::Action, PandaMotionPlanner::State, std::string>> discrepancies_effects;

    // Initialize counters
    int total_plans = results.size();
    int motion_successes = 0;
    int motion_failures = 0;
    int precondition_successes = 0;
    int precondition_failures = 0;
    int effect_successes = 0;
    int effect_failures = 0;
    int black_list_type_1 = 0; // Both motion and precondition failed
    int white_list_type_1 = 0; // Both motion and precondition succeeded
    int black_list_type_2 = 0; // Precondition succeeded, motion failed
    int white_list_type_2 = 0; // Precondition failed, motion succeeded

    for (size_t i = 0; i < total_plans; ++i) {
    bool motion_result = results[i];
    bool precondition_result = precondition_results[i];
    bool effect_result = effect_results[i];

    // Count motion planning successes and failures
    if (motion_result) {
        motion_successes++;
    } else {
        motion_failures++;
    }

    // Count precondition successes and failures
    if (precondition_result) {
        precondition_successes++;
    } else {
        precondition_failures++;
    }

    // Classify into categories based on motion and precondition results
    if (!motion_result && !precondition_result) {
        black_list_type_1++;
    } else if (motion_result && precondition_result) {
        white_list_type_1++;
    } else if (!motion_result && precondition_result) {
        black_list_type_2++;
    } else if (motion_result && !precondition_result) {
        white_list_type_2++;
    }

    // Collect discrepancies for preconditions
    if (motion_result != precondition_result) {
        std::string discrepancy_info;
        if (motion_result && !precondition_result) {
            discrepancy_info = "Motion Planning SUCCEEDED, Precondition Check FAILED";
            discrepancies_preconditions.push_back(std::make_tuple(states[i], actions[i], discrepancy_info));
        } else if (!motion_result && precondition_result) {
            discrepancy_info = "Motion Planning FAILED, Precondition Check SUCCEEDED";
            discrepancies_preconditions.push_back(std::make_tuple(states[i], actions[i], discrepancy_info));
        }
    }

    // For effect results, only consider cases where motion planning and precondition succeeded
    if (motion_result && precondition_result) {
        if (effect_result) {
            effect_successes++;
        } else {
            effect_failures++;
            // Collect discrepancies due to effect mismatches
            discrepancies_effects.push_back(std::make_tuple(states[i], actions[i], resulting_states[i], "Effects not correctly reflected in resulting state"));
        }
    }
    }

    // Print the results
    planner.printResults(states, actions, results, precondition_results, effect_results, resulting_states);
    planner.printSimpleResults(states, actions, results, precondition_results, effect_results, resulting_states);

    // Print discrepancies
    planner.printDiscrepancies(discrepancies_preconditions, discrepancies_effects);


    // Print the metrics
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
    }

    else {
        // Manually define a state
        // State layout: 9 positions in a 3x3 grid and the last element indicates if the gripper is holding a cube (1 = holding, 0 = empty)
        PandaMotionPlanner::State manual_state = {2, 0, 1, 0, 3, 0, 0, 0, 0, 1};  // This means: cube at position 0 and 2, gripper is empty

        // Manually define an action sequence
        // Action Types: PICK (from position), PLACE (to position), STACK (cube on cube), UNSTACK (cube from cube to new position)
        std::vector<PandaMotionPlanner::Action> manual_actions = {
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 0, 3),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::UNSTACK, 2, 1),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 2, 5),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::UNSTACK, 6, 5),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::STACK, 6, 0),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::UNSTACK, 5, 4),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::STACK, 5, 2),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 1),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 1, 7),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 3),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::STACK, 3, 1),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PICK, 4),         // Pick cube from position 0
            PandaMotionPlanner::Action(PandaMotionPlanner::Action::PLACE, 4, 1)         // Pick cube from position 0
        };

        // Run the manual state and action sequence
        bool result = planner.run(manual_actions, manual_state, execute);

        if (result) {
            ROS_INFO("Manual action sequence executed successfully.");
        } else {
            ROS_ERROR("Manual action sequence failed.");
        }
    }
    ros::shutdown();
    return 0;
}
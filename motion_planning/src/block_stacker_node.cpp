#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometric_shapes/shape_operations.h>

static const std::string ARM_PLANNING_GROUP = "panda_arm";
static const std::string GRIPPER_PLANNING_GROUP = "panda_hand";

class PandaMotionPlanner
{
public:
  PandaMotionPlanner()
    : arm_move_group(ARM_PLANNING_GROUP)
    , gripper_move_group(GRIPPER_PLANNING_GROUP)
    , visual_tools("panda_link0")
  {
    arm_move_group.setPlanningTime(10.0);
    
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    // Add the collision object when the planner is initialized
    initializeBasePositions();
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

  void attachObject(const std::string& object_id, const std::string& link_name)
  {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    attached_object.object.id = object_id;
    attached_object.object.operation = attached_object.object.ADD;

    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    ROS_INFO("Attached object '%s' to link '%s'", object_id.c_str(), link_name.c_str());
  }

  void detachObject(const std::string& object_id)
  {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object.id = object_id;
    attached_object.object.operation = attached_object.object.REMOVE;

    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    ROS_INFO("Detached object '%s'", object_id.c_str());
  }

  void planAndExecuteArm(const geometry_msgs::Pose& target_pose)
  {
    arm_move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP));
      visual_tools.trigger();

      visual_tools.prompt("Press 'next' to execute the arm plan");
      arm_move_group.execute(my_plan);
    }
    else
    {
      ROS_ERROR("Arm planning failed");
    }
  }

  void moveGripper(bool open)
{
  std::vector<double> gripper_joint_values;
  if (open)
  {
    gripper_joint_values = {0.04, 0.04};
  }
  else
  {
    gripper_joint_values = {0.025, 0.025};
  }

  gripper_move_group.setJointValueTarget(gripper_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
  bool success = (gripper_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    visual_tools.prompt("Press 'next' to " + std::string(open ? "open" : "close") + " the gripper");
    gripper_move_group.execute(gripper_plan);
  }
  else
  {
    ROS_ERROR("Gripper planning failed");
  }
}

  void allowGripperCubeCollision(const std::string& target_cube_id)
{
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor);
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
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor);
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
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor);
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
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ps->getAllowedCollisionMatrixNonConst();

    acm.setEntry(cube_id1, cube_id2, allow);

    moveit_msgs::PlanningScene planning_scene_msg;
    ps->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    ROS_INFO("%s collision between %s and %s", allow ? "Allowed" : "Disallowed", cube_id1.c_str(), cube_id2.c_str());
}

  void executeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints)
  {
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm_move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction == -1.0)
    {
      ROS_ERROR("Cartesian path planning failed");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    visual_tools.publishTrajectoryLine(plan.trajectory_, arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP));
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to execute the Cartesian path");
    arm_move_group.execute(plan);
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

bool checkIKSolution(const geometry_msgs::Pose& target_pose, const std::string& end_effector_link)
{
    // Get the current state
    robot_state::RobotStatePtr current_state = arm_move_group.getCurrentState();

    // Solve IK
    std::vector<double> joint_values;
    bool found_ik = current_state->setFromIK(
        current_state->getJointModelGroup(ARM_PLANNING_GROUP),
        target_pose,
        end_effector_link,
        10,
        0.1
    );

    if (found_ik) {
        current_state->copyJointGroupPositions(ARM_PLANNING_GROUP, joint_values);
        
        // Print the joint values (x1 in your supervisor's notation)
        ROS_INFO("IK solution (x1): ");
        for (size_t i = 0; i < joint_values.size(); ++i) {
            ROS_INFO("Joint %zu: %f", i, joint_values[i]);
        }

        // Print the target pose (p1 in your supervisor's notation)
        ROS_INFO("Target pose (p1): ");
        ROS_INFO("Position: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
        ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

        return true;
    } else {
        ROS_ERROR("Failed to find IK solution");
        return false;
    }
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

void moveToPosition(const geometry_msgs::Pose& target_pose, const std::string& action_description)
{
    std::string end_effector_link = arm_move_group.getEndEffectorLink();
    ROS_INFO("End effector link: %s", end_effector_link.c_str());

    arm_move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Motion plan succeeded. Ready to move.");
        
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP));
        visual_tools.trigger();

        visual_tools.prompt("Press 'next' to " + action_description);
        
        arm_move_group.execute(my_plan);

        // Log current state
        robot_state::RobotStatePtr current_state = arm_move_group.getCurrentState();
        std::vector<double> current_joint_values;
        current_state->copyJointGroupPositions(ARM_PLANNING_GROUP, current_joint_values);

        ROS_INFO("Current joint values:");
        for (size_t i = 0; i < current_joint_values.size(); ++i) {
            ROS_INFO("Joint %zu: %f", i, current_joint_values[i]);
        }

        geometry_msgs::Pose current_pose = arm_move_group.getCurrentPose().pose;
        ROS_INFO("Current pose:");
        ROS_INFO("Position: x=%f, y=%f, z=%f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
        ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    } else {
        ROS_ERROR("Failed to plan movement to target pose");
    }
}

void pickCube(const std::string& cube_id, const geometry_msgs::Pose& pick_pose)
{
    // Calculate g1 (15 cm above pick_pose)
    geometry_msgs::Pose g1 = calculateIntermediatePosition(pick_pose);

    // Move to g1
    moveToPosition(g1, "move to position above " + cube_id);

    // Lower to pick position
    geometry_msgs::Pose lower_pose = pick_pose;
    lower_pose.position.z = pick_pose.position.z + 0.1;  // Slightly above the cube to account for gripper size
    moveToPosition(lower_pose, "lower to pick " + cube_id);

    // Gripper actions
    moveGripper(true);  // Open gripper
    visual_tools.prompt("Press 'next' to close gripper and pick " + cube_id);
    moveGripper(false);  // Close gripper
    attachObject(cube_id, "panda_hand");

    // Move back to g1
    moveToPosition(g1, "lift " + cube_id);
}

void placeCube(const std::string& cube_id, const geometry_msgs::Pose& place_pose)
{
    // Calculate g2 (15 cm above place_pose)
    geometry_msgs::Pose g2 = calculateIntermediatePosition(place_pose);

    // Move to g2
    moveToPosition(g2, "move to position above placement for " + cube_id);

    // Lower to place position
    geometry_msgs::Pose lower_pose = place_pose;
    lower_pose.position.z = place_pose.position.z + 0.12;  // Slightly above the target position
    moveToPosition(lower_pose, "lower to place " + cube_id);

    // Gripper actions
    visual_tools.prompt("Press 'next' to open gripper and release " + cube_id);
    moveGripper(true);  // Open gripper
    detachObject(cube_id);

    // Move back to g2
    moveToPosition(g2, "lift after placing " + cube_id);
}

struct Action {
    enum Type { PICK, PLACE, STACK, UNSTACK };
    Type type;
    int cube_index1;  // For PICK, PLACE, STACK (cube to move), and UNSTACK (cube to move)
    int cube_index2;  // For STACK (cube to stack on), and UNSTACK (cube to unstack from)
    int position_index;  // For PLACE and UNSTACK (position to place the cube)

    Action(Type t, int index1) : type(t), cube_index1(index1), cube_index2(-1), position_index(index1) {}
    Action(Type t, int index1, int index2) : type(t), cube_index1(index1), cube_index2(index2), position_index(-1) {}
    Action(Type t, int index1, int index2, int pos) : type(t), cube_index1(index1), cube_index2(index2), position_index(pos) {}
};

void run(const std::vector<Action>& action_sequence)
{
    // Define the initial state
    State initial_state = {0, 1, 0, 0, 2, 0, 0, 0, 1, 1};  // Last element is gripper state
    addCollisionObjects(initial_state);
    
    // Allow collisions between cubes and tables
    allowCubeCollisions(true);
    
    GoalPositions positions = defineGoalPositions();
    
    // Get all cube poses
    std::vector<std::pair<std::string, geometry_msgs::Pose>> cube_poses = getAllCubePoses();
    
    // Ensure we have the correct number of cubes
    if (cube_poses.size() != 5) {
        ROS_ERROR("Expected 5 cubes, but found %zu", cube_poses.size());
        return;
    }

    // Create a dynamic state representation
    std::vector<int> dynamic_state(10, 0);
    for (size_t i = 0; i < initial_state.size(); ++i) {
        dynamic_state[i] = initial_state[i];
    }

    std::string current_cube_id;
    std::string target_cube_id;
    geometry_msgs::Pose current_cube_pose;
    int current_pick_position = -1;
    bool gripper_has_cube = dynamic_state[9] == 1;

    // If gripper starts with a cube, we need to set up the initial cube and collision states
if (gripper_has_cube) {
    current_cube_id = "small_cube0";  // Using the new naming convention
    
    // Close the gripper
    moveGripper(false);  // Assuming false means close

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
    moveGripper(true);  // Assuming true means open

    // Disallow all gripper-cube collisions at the start if gripper is empty
    disallowAllGripperCubeCollisions();

    ROS_INFO("Gripper initialized in open position without cube");
}

    for (const auto& action : action_sequence) {
    switch (action.type) {
        case Action::PICK:
        {
            if (gripper_has_cube) {
                ROS_ERROR("Cannot pick a cube. Gripper is already holding a cube.");
                continue;
            }
            if (action.cube_index1 >= cube_poses.size()) {
                ROS_ERROR("Invalid cube index: %d", action.cube_index1);
                continue;
            }
            current_cube_id = cube_poses[action.cube_index1].first;
            current_cube_pose = cube_poses[action.cube_index1].second;
            
            // Find the position of the cube being picked
            for (size_t i = 0; i < base_positions.size(); ++i) {
                if (std::abs(current_cube_pose.position.x - (positions.table_center_pose.position.x + base_positions[i].x)) < 0.01 &&
                    std::abs(current_cube_pose.position.y - (positions.table_center_pose.position.y + base_positions[i].y)) < 0.01) {
                    current_pick_position = i;
                    break;
                }
            }

            if (current_pick_position != -1) {
                dynamic_state[current_pick_position]--;
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
            // Set orientation for pick pose
            tf2::Quaternion q;
            q.setRPY(-M_PI, 0, -M_PI/4);  // Roll, Pitch, Yaw
            pick_pose.orientation = tf2::toMsg(q);
            pickCube(current_cube_id, pick_pose);

            gripper_has_cube = true;
            dynamic_state[9] = 1;  // Update gripper state
            break;
        }
        case Action::PLACE:
        {
            if (!gripper_has_cube) {
                ROS_ERROR("Cannot place a cube. Gripper is not holding any cube.");
                continue;
            }
            geometry_msgs::Pose place_pose = positions.table_center_pose;
            place_pose.position.x += base_positions[action.position_index].x;
            place_pose.position.y += base_positions[action.position_index].y;
            place_pose.position.z = base_positions[action.position_index].z;
            
            // Adjust height based on the current stack at this position
            place_pose.position.z += dynamic_state[action.position_index] * 0.05;

            // Check if we're placing on another cube
            if (dynamic_state[action.position_index] > 0) {
                // Find the ID of the cube we're placing on
                double expected_height = base_positions[action.position_index].z + (dynamic_state[action.position_index] - 1) * 0.05;

                for (const auto& cube : cube_poses) {
                    geometry_msgs::Pose cube_pose = cube.second;
                    if (std::abs(cube_pose.position.x - (positions.table_center_pose.position.x + base_positions[action.position_index].x)) < 0.01 &&
                        std::abs(cube_pose.position.y - (positions.table_center_pose.position.y + base_positions[action.position_index].y)) < 0.01 &&
                        std::abs(cube_pose.position.z - expected_height) < 0.01) {
                        target_cube_id = cube.first;
                        break;
                    }
                }

                // Allow collision between the current cube and the target cube
                allowSpecificCubeCollision(current_cube_id, target_cube_id, true);
            }

            // Set orientation for place pose
            tf2::Quaternion q;
            q.setRPY(-M_PI, 0, -M_PI/4);  // Roll, Pitch, Yaw
            place_pose.orientation = tf2::toMsg(q);
            placeCube(current_cube_id, place_pose);

            // Update the dynamic state
            dynamic_state[action.position_index]++;

            // After placing, disallow collision with all cubes except the one it's stacked on
            disallowAllGripperCubeCollisions();
            for (const auto& cube : cube_poses) {
                if (cube.first != current_cube_id) {
                    allowSpecificCubeCollision(current_cube_id, cube.first, cube.first == target_cube_id);
                }
            }

            // Update the cube's pose in cube_poses
            for (auto& cube : cube_poses) {
                if (cube.first == current_cube_id) {
                    cube.second = place_pose;
                    break;
                }
            }

            current_cube_id.clear();
            target_cube_id.clear();
            current_pick_position = -1;
            gripper_has_cube = false;
            dynamic_state[9] = 0;  // Update gripper state
            break;
        }
        case Action::STACK:
{
    if (gripper_has_cube) {
        ROS_ERROR("Cannot stack a cube. Gripper is already holding a cube.");
        continue;
    }
    if (action.cube_index1 >= cube_poses.size() || action.cube_index2 >= cube_poses.size()) {
        ROS_ERROR("Invalid cube indices for STACK action");
        continue;
    }
    std::string cube_to_move = cube_poses[action.cube_index1].first;
    std::string cube_to_stack_on = cube_poses[action.cube_index2].first;
    geometry_msgs::Pose stack_on_pose = cube_poses[action.cube_index2].second;

    // Check if the cube to stack on already has something on top
bool can_stack = false;
int stack_position = -1;
double expected_height = 0;

for (size_t i = 0; i < base_positions.size(); ++i) {
    if (std::abs(stack_on_pose.position.x - (positions.table_center_pose.position.x + base_positions[i].x)) < 0.01 &&
        std::abs(stack_on_pose.position.y - (positions.table_center_pose.position.y + base_positions[i].y)) < 0.01) {
        stack_position = i;
        expected_height = base_positions[i].z + (dynamic_state[i] - 1) * 0.05;
        break;
    }
}

if (stack_position != -1) {
    // Check if the cube we want to stack on is at the top of its stack
    if (std::abs(stack_on_pose.position.z - expected_height) < 0.01) {
        can_stack = true;
    } else {
        can_stack = false;
        ROS_ERROR("Cannot stack on cube %s, it already has a cube on top", cube_to_stack_on.c_str());
    }
} else {
    ROS_ERROR("Could not find the position of the cube to stack on");
    can_stack = false;
}

if (can_stack && stack_position != -1) {
    // Pick phase
    current_cube_id = cube_to_move;
    current_cube_pose = cube_poses[action.cube_index1].second;

    // Allow collision only with the cube we're about to pick
    allowGripperCubeCollision(current_cube_id);

    // Disallow collisions between the picked cube and all other cubes
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, false);
        }
    }

    // Prepare the pick pose
    geometry_msgs::Pose pick_pose = current_cube_pose;
    pick_pose.position.z += 0.02;  // Slightly above the cube

    // Set orientation for pick pose
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI/4);  // Roll, Pitch, Yaw
    pick_pose.orientation = tf2::toMsg(q);

    // Pick up the cube to move
    pickCube(cube_to_move, pick_pose);

    gripper_has_cube = true;
    dynamic_state[9] = 1;  // Update gripper state

    // Place phase
    // Calculate the new position (5cm above the cube to stack on)
    geometry_msgs::Pose stack_pose = stack_on_pose;
    stack_pose.position.z += 0.05;

    // Allow collision between the current cube and the target cube
    allowSpecificCubeCollision(current_cube_id, cube_to_stack_on, true);

    // Place the cube on top of the other
    placeCube(cube_to_move, stack_pose);

    // After placing, disallow collision with all cubes except the one it's stacked on
    disallowAllGripperCubeCollisions();
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, cube.first == cube_to_stack_on);
        }
    }

    // Update dynamic state and cube poses
    dynamic_state[stack_position]++;
    cube_poses[action.cube_index1].second = stack_pose;

    current_cube_id.clear();
    gripper_has_cube = false;
    dynamic_state[9] = 0;  // Update gripper state
} else {
    ROS_ERROR("Cannot perform stack action due to invalid conditions");
}
    break;
    }
        case Action::UNSTACK:
{
    if (gripper_has_cube) {
        ROS_ERROR("Cannot unstack a cube. Gripper is already holding a cube.");
        continue;
    }
    if (action.cube_index1 >= cube_poses.size() || action.cube_index2 >= cube_poses.size()) {
        ROS_ERROR("Invalid cube indices for UNSTACK action");
        continue;
    }
    std::string cube_to_move = cube_poses[action.cube_index1].first;
    std::string cube_to_unstack_from = cube_poses[action.cube_index2].first;
    geometry_msgs::Pose unstack_from_pose = cube_poses[action.cube_index2].second;

    // Check if the cube to unstack is actually on top of the other cube
    bool can_unstack = false;
    int unstack_position = -1;
    for (size_t i = 0; i < base_positions.size(); ++i) {
        if (std::abs(unstack_from_pose.position.x - (positions.table_center_pose.position.x + base_positions[i].x)) < 0.01 &&
            std::abs(unstack_from_pose.position.y - (positions.table_center_pose.position.y + base_positions[i].y)) < 0.01) {
            if (dynamic_state[i] > 1) {
                can_unstack = true;
                unstack_position = i;
            }
            break;
        }
    }

    if (can_unstack && unstack_position != -1) {
    // Pick phase
    current_cube_id = cube_to_move;
    current_cube_pose = cube_poses[action.cube_index1].second;

    // Allow collision only with the cube we're about to pick
    allowGripperCubeCollision(current_cube_id);

    // Disallow collisions between the picked cube and all other cubes
    for (const auto& cube : cube_poses) {
        if (cube.first != current_cube_id) {
            allowSpecificCubeCollision(current_cube_id, cube.first, false);
        }
    }

    // Prepare the pick pose
    geometry_msgs::Pose pick_pose = current_cube_pose;
    pick_pose.position.z += 0.02;  // Slightly above the cube

    // Set orientation for pick pose
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI/4);  // Roll, Pitch, Yaw
    pick_pose.orientation = tf2::toMsg(q);

    // Pick up the cube to move
    pickCube(cube_to_move, pick_pose);

    gripper_has_cube = true;
    dynamic_state[9] = 1;  // Update gripper state


        // Place phase
        // Calculate the new position on the table
        geometry_msgs::Pose place_pose = positions.table_center_pose;
        place_pose.position.x += base_positions[action.position_index].x;
        place_pose.position.y += base_positions[action.position_index].y;
        place_pose.position.z = base_positions[action.position_index].z + dynamic_state[action.position_index] * 0.05;

        // Check if we're placing on another cube
        std::string target_cube_id;
        if (dynamic_state[action.position_index] > 0) {
            // Find the ID of the cube we're placing on (if any)
            for (const auto& cube : cube_poses) {
                geometry_msgs::Pose cube_pose = cube.second;
                if (std::abs(cube_pose.position.x - place_pose.position.x) < 0.01 &&
                    std::abs(cube_pose.position.y - place_pose.position.y) < 0.01 &&
                    std::abs(cube_pose.position.z - (place_pose.position.z - 0.05)) < 0.01) {
                    target_cube_id = cube.first;
                    break;
                }
            }

            if (!target_cube_id.empty()) {
                // Allow collision between the current cube and the target cube
                allowSpecificCubeCollision(current_cube_id, target_cube_id, true);
            }
        }

        // Place the cube on the table
        placeCube(cube_to_move, place_pose);

        // After placing, disallow collision with all cubes except the one it's stacked on (if any)
        disallowAllGripperCubeCollisions();
        for (const auto& cube : cube_poses) {
            if (cube.first != current_cube_id) {
                allowSpecificCubeCollision(current_cube_id, cube.first, cube.first == target_cube_id);
            }
        }

        // Update dynamic state and cube poses
        dynamic_state[unstack_position]--;
        dynamic_state[action.position_index]++;
        cube_poses[action.cube_index1].second = place_pose;

        current_cube_id.clear();
        gripper_has_cube = false;
        dynamic_state[9] = 0;  // Update gripper state
    } else {
        ROS_ERROR("Cannot unstack cube %s from cube %s, they are not stacked", cube_to_move.c_str(), cube_to_unstack_from.c_str());
    }
    break;
}
    }

    // Update current pose after each action
    geometry_msgs::Pose current_pose = arm_move_group.getCurrentPose().pose;

    // Print the current dynamic state for debugging
    ROS_INFO("Current dynamic state:");
    for (size_t i = 0; i < 9; ++i) {
        ROS_INFO("Position %zu: %d cubes", i, dynamic_state[i]);
    }
    ROS_INFO("Gripper state: %s", gripper_has_cube ? "Holding cube" : "Empty");
}

    // Return to starting position
    moveToPosition(positions.start_pose, "return to the starting position");
    visual_tools.prompt("Press 'next' to end the demo");
}

  struct CubePosition {
        double x, y, z;
        CubePosition(double x, double y, double z) : x(x), y(y), z(z) {}
    };

    typedef std::array<int, 10> State;  // 9 positions + 1 gripper state

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

    // Create Table 2
    moveit_msgs::CollisionObject table2;
    table2.header.frame_id = "panda_link0";
    table2.id = "table2";
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions = {0.25, 0.175, 0.4};  // 25x17.5x40 cm
    geometry_msgs::Pose table2_pose;
    table2_pose.orientation.w = 1.0;
    table2_pose.position.x = goal_pose2.position.x;
    table2_pose.position.y = goal_pose2.position.y;
    table2_pose.position.z = 0.2;  // Half of table height
    table2.primitives.push_back(primitive2);
    table2.primitive_poses.push_back(table2_pose);
    table2.operation = table2.ADD;
    collision_objects.push_back(table2);

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

private:
  moveit::planning_interface::MoveGroupInterface arm_move_group;
  moveit::planning_interface::MoveGroupInterface gripper_move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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

    PandaMotionPlanner planner;

    // Define your custom action sequence
    std::vector<PandaMotionPlanner::Action> action_sequence = {
        {PandaMotionPlanner::Action::PLACE, 2},  // Place at position 1
        {PandaMotionPlanner::Action::STACK, 1, 0},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::STACK, 3, 1},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::STACK, 2, 3},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::STACK, 4, 2},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::UNSTACK, 4, 2, 5},   // Pick cube 2
        {PandaMotionPlanner::Action::STACK, 2, 4},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::STACK, 3, 2},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::STACK, 1, 3},  // Stack cube 1 on cube 0
        {PandaMotionPlanner::Action::STACK, 0, 1},  // Stack cube 1 on cube 0
        // {PandaMotionPlanner::Action::PICK, 1},   // Pick cube 1
        // {PandaMotionPlanner::Action::PLACE, 2},  // Place at position 3
        // {PandaMotionPlanner::Action::PICK, 3},   // Pick cube 3
        // {PandaMotionPlanner::Action::PLACE, 2},  // Place at position 3
        // {PandaMotionPlanner::Action::PICK, 4},   // Pick cube 4
        // {PandaMotionPlanner::Action::PLACE, 2},  // Place at position 5
        // {PandaMotionPlanner::Action::PICK, 2},   // Pick cube 4
        // {PandaMotionPlanner::Action::PLACE, 5},  // Place at position 7
        // {PandaMotionPlanner::Action::UNSTACK, 4, 3, 5},   // Pick cube 2
        // {PandaMotionPlanner::Action::PLACE, 5},  // Place at position 3
        // Add more actions as needed
    };

    planner.run(action_sequence);

    ros::shutdown();
    return 0;
}
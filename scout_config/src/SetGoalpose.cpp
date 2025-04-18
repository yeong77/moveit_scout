#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_set_goal_pose_cpp");
    ros::NodeHandle node_handle("~");  // private namespace

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("scout_group");

    // Get parameters from launch file
    double start_x, start_y, start_yaw;
    double goal_x, goal_y, goal_yaw;

    node_handle.param("start_x", start_x, 0.0);
    node_handle.param("start_y", start_y, 0.0);
    node_handle.param("start_yaw", start_yaw, 0.0);

    node_handle.param("goal_x", goal_x, 5.0);
    node_handle.param("goal_y", goal_y, 5.0);
    node_handle.param("goal_yaw", goal_yaw, 0.037);

    // Start state
    robot_state::RobotState start_state(*move_group.getCurrentState());
    std::vector<double> start_joint_values = {start_x, start_y, start_yaw};
    start_state.setJointPositions("virtual_joint", start_joint_values);
    move_group.setStartState(start_state);

    // Goal state
    move_group.setJointValueTarget("virtual_joint", {goal_x, goal_y, goal_yaw});

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Planning successful.");
    } else {
        ROS_WARN("Planning failed.");
    }

    ros::shutdown();
    return 0;
}


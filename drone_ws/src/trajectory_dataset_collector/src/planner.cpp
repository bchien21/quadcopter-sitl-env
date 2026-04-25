#include <memory>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

// ros2 run tf2_ros static_transform_publisher --x 0.12 --y 0.03 --z 0.0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id camera_link


moveit_msgs::msg::Constraints setRotationConstraints(std::vector<std::string> jointNames) {

    moveit_msgs::msg::Constraints c;
  
    // Lock roll & pitch ~= 0 (allow any yaw encoded in quat)
    for (auto name : jointNames) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = name;
      jc.position = 0.0;
      jc.tolerance_above = 1e-3;
      jc.tolerance_below = 1e-3;
      jc.weight = 1.0;
      c.joint_constraints.push_back(jc);
    }
  
    return c;
  
}

// Generate a random joint target within workspace bounds
std::map<std::string, double> sampleRandomGoal(
    std::mt19937& rng,
    double x_min, double x_max,
    double y_min, double y_max)
{
  std::uniform_real_distribution<double> dist_x(x_min, x_max);
  std::uniform_real_distribution<double> dist_y(y_min, y_max);

  return {
    {"virtual/trans_x", dist_x(rng)},
    {"virtual/trans_y", dist_y(rng)},
    {"virtual/trans_z", 0.0},
    {"virtual/rot_x",   0.0},
    {"virtual/rot_y",   0.0},
    {"virtual/rot_z",   0.0},
    {"virtual/rot_w",   1.0},
  };
}

// Check if a joint target is collision-free using the planning scene
bool isGoalValid(
    const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
    const std::string& group_name,
    const std::map<std::string, double>& joint_target)
{
  planning_scene_monitor::LockedPlanningSceneRO scene(psm);

  // Copy the current robot state and apply the candidate goal
  moveit::core::RobotState goal_state = scene->getCurrentState();
  for (const auto& [name, value] : joint_target) {
    goal_state.setVariablePosition(name, value);
  }
  goal_state.update();

  // Check collision for the goal state
  collision_detection::CollisionRequest req;
  req.group_name = group_name;
  collision_detection::CollisionResult res;
  scene->checkCollision(req, res, goal_state);

  return !res.collision;
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto move_group_interface = MoveGroupInterface(node, "quad-base");
  move_group_interface.setPlanningTime(30.0);
  move_group_interface.setWorkspace(0.00, -3.0, -0.01, 8.0, 3.0, 0.01);

  // Set Constraints
  moveit_msgs::msg::Constraints constraints = setRotationConstraints({"virtual/rot_x","virtual/rot_y"});
  move_group_interface.setPathConstraints(constraints);

  // Set up Planning Scene Monitor for collision checking
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  psm->startSceneMonitor("/move_group/monitored_planning_scene");
  psm->startWorldGeometryMonitor();
  psm->requestPlanningSceneState("/get_planning_scene");

  // Wait for the planning scene to be populated
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(logger, "Planning scene monitor initialized");

  // Random number generator
  std::mt19937 rng(std::random_device{}());

  // Hardcoded goal for testing (switch back to sampleRandomGoal for dataset collection)
  std::map<std::string, double> joint_target{
    {"virtual/trans_x", 7.5},
    {"virtual/trans_y", 2.5},
    {"virtual/trans_z", 0.0},
    {"virtual/rot_x",   0.0},
    {"virtual/rot_y",   0.0},
    {"virtual/rot_z",   0.0},
    {"virtual/rot_w",   1.0},
  };

  RCLCPP_INFO(logger, "Goal: (%.2f, %.2f, %.2f)",
              joint_target["virtual/trans_x"],
              joint_target["virtual/trans_y"],
              joint_target["virtual/trans_z"]);

  // Check if the goal is collision-free
  if (!isGoalValid(psm, "quad-base", joint_target)) {
    RCLCPP_ERROR(logger, "Goal is in collision!");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "Goal is collision-free");

  // Plan to the goal
  move_group_interface.setJointValueTarget(joint_target);

  bool planned = false;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto ok = static_cast<bool>(move_group_interface.plan(msg));

  if (ok) {
    plan = msg;
    planned = true;
    RCLCPP_INFO(logger, "Planning succeeded in %.3f s", plan.planning_time_);
  } else {
    RCLCPP_ERROR(logger, "Planning failed");
  }


  if (planned) {

    RCLCPP_INFO(logger, "Final goal: (%.2f, %.2f, %.2f)",
                joint_target["virtual/trans_x"],
                joint_target["virtual/trans_y"],
                joint_target["virtual/trans_z"]);

    const auto& md = plan.trajectory_.multi_dof_joint_trajectory;

    // Helper to get seconds as double
    auto to_sec = [](const builtin_interfaces::msg::Duration& d) {
      return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
    };

    // --- Multi-DOF waypoints (typical for floating-base drones) ---
    if (!md.points.empty()) {
      RCLCPP_INFO(logger, "Multi-DOF trajectory: %zu points, %zu joints",
                  md.points.size(), md.joint_names.size());
      for (size_t i = 0; i < md.points.size(); ++i) {

        const auto& p = md.points[i];
        double t = to_sec(p.time_from_start);

        for (size_t j = 0; j < p.transforms.size(); ++j) {

          const auto& name = (j < md.joint_names.size()) ? md.joint_names[j] : std::string("?");
          const auto& tf = p.transforms[j];

          RCLCPP_INFO(
            logger,
            "[%03zu] t=%.3f  %s  xyz=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
            i, t, name.c_str(),
            tf.translation.x, tf.translation.y, tf.translation.z,
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
          );
        }

      }
    }

  } else {
    // RCLCPP_ERROR(logger, "Failed to find a valid plan after %d attempts", max_attempts);
    RCLCPP_ERROR(logger, "Failed to find a valid plan");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setWorkspace(0.00, -2.0, -0.01, 5.0, 2.0, 0.01);

  // Set Constraints
  moveit_msgs::msg::Constraints constraints = setRotationConstraints({"virtual/rot_x","virtual/rot_y"});
  move_group_interface.setPathConstraints(constraints);

  // --- joint-space goal (floating base) ---
  std::map<std::string, double> joint_target{
    {"virtual/trans_x", 3.5},
    {"virtual/trans_y", 1.5},
    {"virtual/trans_z", 0.0},
    {"virtual/rot_x",   0.0},
    {"virtual/rot_y",   0.0},
    {"virtual/rot_z",   0.0},
    {"virtual/rot_w",   1.0},
  };
  move_group_interface.setJointValueTarget(joint_target);


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();


  if (success) {

    RCLCPP_INFO(logger, "Planning succeeded in %.3f s", plan.planning_time_);

    const auto& jt = plan.trajectory_.joint_trajectory;
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
    RCLCPP_ERROR(logger, "Planning failed or timed out");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

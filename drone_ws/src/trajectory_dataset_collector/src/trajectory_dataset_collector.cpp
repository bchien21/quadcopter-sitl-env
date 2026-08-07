#include <memory>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <H5Cpp.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


struct TrajectoryPoint
{
  double x;
  double y;
  double z;
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct Trajectory
{
  std::vector<TrajectoryPoint> points;
};

class TrajectoryDatasetCollector : public rclcpp::Node
{
public:
    TrajectoryDatasetCollector()
    : Node("trajectory_dataset_collector"),
      logger_(get_logger())
    {

      timer_ = create_wall_timer(
      1000ms, std::bind(&TrajectoryDatasetCollector::collect_trajectory, this));

      num_samples_ = declare_parameter<int>("num_samples_", 200);
      x_min_ = declare_parameter<double>("x_min", -10.0);
      x_max_ = declare_parameter<double>("x_max", 10.0);
      y_min_ = declare_parameter<double>("y_min", -10.0);
      y_max_ = declare_parameter<double>("y_max", 10.0);
      z_min_ = declare_parameter<double>("z_min", 0.5);
      z_max_ = declare_parameter<double>("z_max", 5.0);
      dataset_file_ = std::make_unique<H5::H5File>(
        declare_parameter<std::string>("dataset_path", "trajectories.h5"), H5F_ACC_TRUNC);

    }

    void initialize_move_group()
    {
      move_group_interface_ = std::make_unique<MoveGroupInterface>(
        shared_from_this(), "quad-base");
      move_group_interface_->setPathConstraints(rotation_constraints());

      planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), "robot_description");
      planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
    }

private:

    void collect_trajectory() {

      TrajectoryPoint goal = sample_goal();
      Trajectory trajectory = plan_trajectory(goal);

      if (trajectory.points.empty()) {
        RCLCPP_WARN(logger_, "Planning failed; sampling another goal");
        return;
      }

      H5::Group sample = dataset_file_->createGroup("sample_" + std::to_string(samples_collected_));
      const double goal_data[] = {goal.x, goal.y, goal.z};
      hsize_t goal_shape[] = {3};
      H5::DataSpace goal_space(1, goal_shape);
      sample.createDataSet("goal", H5::PredType::NATIVE_DOUBLE, goal_space)
        .write(goal_data, H5::PredType::NATIVE_DOUBLE);

      std::vector<double> path_data;
      for (const auto & point : trajectory.points) {
        path_data.insert(path_data.end(), {point.x, point.y, point.z});
      }
      hsize_t path_shape[] = {trajectory.points.size(), 3};
      H5::DataSpace path_space(2, path_shape);
      sample.createDataSet("trajectory", H5::PredType::NATIVE_DOUBLE, path_space)
        .write(path_data.data(), H5::PredType::NATIVE_DOUBLE);
      dataset_file_->flush(H5F_SCOPE_GLOBAL);

      RCLCPP_INFO(logger_, "Saved trajectory %d with %zu points",
        ++samples_collected_, trajectory.points.size());
        
      if (samples_collected_ >= num_samples_) {
        timer_->cancel();
        rclcpp::shutdown();
      }

    }

    TrajectoryPoint sample_goal() {

        static std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> x_dist(x_min_, x_max_);
        std::uniform_real_distribution<double> y_dist(y_min_, y_max_);
        std::uniform_real_distribution<double> z_dist(z_min_, z_max_);

        while (true) {
            TrajectoryPoint goal{
                x_dist(rng), y_dist(rng), z_dist(rng)};

            planning_scene_monitor::LockedPlanningSceneRO scene(
                planning_scene_monitor_);
            moveit::core::RobotState goal_state = scene->getCurrentState();
            goal_state.setVariablePosition("virtual/trans_x", goal.x);
            goal_state.setVariablePosition("virtual/trans_y", goal.y);
            goal_state.setVariablePosition("virtual/trans_z", goal.z);
            goal_state.setVariablePosition("virtual/rot_x", goal.qx);
            goal_state.setVariablePosition("virtual/rot_y", goal.qy);
            goal_state.setVariablePosition("virtual/rot_z", goal.qz);
            goal_state.setVariablePosition("virtual/rot_w", goal.qw);
            goal_state.update();

            collision_detection::CollisionRequest request;
            collision_detection::CollisionResult result;
            request.group_name = "quad-base";
            scene->checkCollision(request, result, goal_state);

            if (!result.collision) {
                return goal;
            }
        }

    }


    moveit_msgs::msg::Constraints rotation_constraints() {
      moveit_msgs::msg::Constraints constraints;
      moveit_msgs::msg::OrientationConstraint rotation;
      rotation.header.frame_id = "world";
      rotation.link_name = "base_link";
      rotation.orientation.w = 1.0;
      rotation.absolute_x_axis_tolerance = 1e-3;
      rotation.absolute_y_axis_tolerance = 1e-3;
      rotation.absolute_z_axis_tolerance = 1e-3;
      rotation.weight = 1.0;
      constraints.orientation_constraints.push_back(rotation);
      return constraints;
    }

    Trajectory plan_trajectory(const TrajectoryPoint& goal) {
      move_group_interface_->setStartStateToCurrentState();
      move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
      move_group_interface_->setJointValueTarget(std::map<std::string, double>{
        {"virtual/trans_x", goal.x}, {"virtual/trans_y", goal.y},
        {"virtual/trans_z", goal.z}, {"virtual/rot_x", goal.qx},
        {"virtual/rot_y", goal.qy}, {"virtual/rot_z", goal.qz},
        {"virtual/rot_w", goal.qw}});

      MoveGroupInterface::Plan plan;
      if (!move_group_interface_->plan(plan)) {
        return {};
      }

      Trajectory trajectory;
      for (const auto & point : plan.trajectory_.multi_dof_joint_trajectory.points) {
        if (!point.transforms.empty()) {
          const auto & pose = point.transforms.front();
          trajectory.points.push_back({
            pose.translation.x, pose.translation.y, pose.translation.z,
            pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w});
        }
      }
      return trajectory;

    }

    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr timer_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::unique_ptr<MoveGroupInterface> move_group_interface_;
    std::unique_ptr<H5::H5File> dataset_file_;
    int num_samples_;
    int samples_collected_{0};
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryDatasetCollector>();
  node->initialize_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

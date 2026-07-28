#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

class OctomapMappingMission final : public rclcpp::Node
{
public:
  OctomapMappingMission()
  : Node("octomap_mapping_mission")
  {
    const rmw_qos_profile_t sensor_profile = rmw_qos_profile_sensor_data;
    const auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(sensor_profile.history, sensor_profile.depth),
      sensor_profile);

    offboard_control_mode_publisher_ =
      create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
    trajectory_setpoint_publisher_ =
      create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
    vehicle_command_publisher_ =
      create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);
    manual_setpoint_subscription_ = create_subscription<TrajectorySetpoint>(
      "/octomap_mapping_mission/manual_setpoint", rclcpp::QoS(10),
      [this](const TrajectorySetpoint::SharedPtr message) {
        receive_manual_setpoint(*message);
      });

    timer_ = create_wall_timer(100ms, [this]() {timer_callback();});

    RCLCPP_INFO(
      get_logger(),
      "Mapping mission ready. Arm in QGroundControl; Offboard mode will be "
      "requested after one second of setpoint streaming.");
    RCLCPP_INFO(
      get_logger(),
      "After the four-waypoint sequence, publish px4_msgs/msg/TrajectorySetpoint "
      "commands on /octomap_mapping_mission/manual_setpoint.");
  }

private:
  struct Waypoint
  {
    std::array<float, 3> position_ned;
    float yaw_ned;
  };

  // PX4 local NED coordinates: +X north, +Y east, +Z down.
  //
  // In the Gazebo ENU world these form a 4 m square at 2.5 m altitude:
  //   (east=0, north=0), (east=0, north=4),
  //   (east=4, north=4), (east=4, north=0).
  //
  // Verify that this square is collision-free in the active Gazebo world
  // before running the node.
  static constexpr std::array<Waypoint, 4> kWaypoints{{
    {{{0.0F, 0.0F, -2.5F}}, 0.0F},
    {{{4.0F, 0.0F, -2.5F}}, 0.0F},
    {{{4.0F, 4.0F, -2.5F}}, 1.5707963F},
    {{{0.0F, 4.0F, -2.5F}}, 3.1415927F},
  }};

  static constexpr uint64_t kOffboardRequestTick = 10;
  static constexpr uint64_t kTicksPerWaypoint = 50;
  static constexpr uint64_t kManualControlStartTick =
    kTicksPerWaypoint * (kWaypoints.size() - 1);

  void timer_callback()
  {
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    // PX4 requires OffboardControlMode and TrajectorySetpoint messages to be
    // streamed before accepting the mode change. The vehicle is intentionally
    // not armed here; arm it in QGroundControl before running this node.
    if (offboard_setpoint_counter_ == kOffboardRequestTick) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      RCLCPP_INFO(get_logger(), "Requested PX4 Offboard mode");
    }

    ++offboard_setpoint_counter_;
  }

  std::size_t current_waypoint_index() const
  {
    return std::min<std::size_t>(
      offboard_setpoint_counter_ / kTicksPerWaypoint, kWaypoints.size() - 1);
  }

  bool scheduled_sequence_complete() const
  {
    return offboard_setpoint_counter_ >= kManualControlStartTick;
  }

  void receive_manual_setpoint(const TrajectorySetpoint & message)
  {
    if (!scheduled_sequence_complete()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring manual setpoint: the four-waypoint sequence is still active");
      return;
    }

    const bool position_is_finite =
      std::all_of(
      message.position.begin(), message.position.end(), [](float value) {
        return std::isfinite(value);
      });
    if (!position_is_finite || !std::isfinite(message.yaw)) {
      RCLCPP_ERROR(
        get_logger(), "Ignoring manual setpoint: position and yaw must all be finite");
      return;
    }

    manual_waypoint_.position_ned = message.position;
    manual_waypoint_.yaw_ned = message.yaw;
    manual_setpoint_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Accepted manual setpoint: NED=(%.1f, %.1f, %.1f), yaw=%.2f rad",
      manual_waypoint_.position_ned[0],
      manual_waypoint_.position_ned[1],
      manual_waypoint_.position_ned[2],
      manual_waypoint_.yaw_ned);
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode message{};
    message.position = true;
    message.velocity = false;
    message.acceleration = false;
    message.attitude = false;
    message.body_rate = false;
    message.timestamp = timestamp_us();
    offboard_control_mode_publisher_->publish(message);
  }

  void publish_trajectory_setpoint()
  {
    const std::size_t waypoint_index = current_waypoint_index();
    const bool use_manual_setpoint =
      scheduled_sequence_complete() && manual_setpoint_received_;
    const auto & waypoint =
      use_manual_setpoint ? manual_waypoint_ : kWaypoints[waypoint_index];

    if (!use_manual_setpoint && waypoint_index != last_logged_waypoint_index_) {
      RCLCPP_INFO(
        get_logger(),
        "Commanding waypoint %zu/%zu: NED=(%.1f, %.1f, %.1f), yaw=%.2f rad",
        waypoint_index + 1, kWaypoints.size(),
        waypoint.position_ned[0],
        waypoint.position_ned[1],
        waypoint.position_ned[2],
        waypoint.yaw_ned);
      last_logged_waypoint_index_ = waypoint_index;
    }

    TrajectorySetpoint message{};
    message.position = waypoint.position_ned;
    message.yaw = waypoint.yaw_ned;
    message.timestamp = timestamp_us();
    trajectory_setpoint_publisher_->publish(message);
  }

  void publish_vehicle_command(
    uint16_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    VehicleCommand message{};
    message.param1 = param1;
    message.param2 = param2;
    message.command = command;
    message.target_system = 1;
    message.target_component = 1;
    message.source_system = 1;
    message.source_component = 1;
    message.from_external = true;
    message.timestamp = timestamp_us();
    vehicle_command_publisher_->publish(message);
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr manual_setpoint_subscription_;

  uint64_t offboard_setpoint_counter_{0};
  std::size_t last_logged_waypoint_index_{std::numeric_limits<std::size_t>::max()};
  Waypoint manual_waypoint_{kWaypoints.back()};
  bool manual_setpoint_received_{false};
};

int main(int argc, char * argv[])
{
  std::cout << "Starting deterministic OctoMap mapping mission..." << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapMappingMission>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

class OctomapMappingMission : public rclcpp::Node
{
public:
  OctomapMappingMission()
  : Node("octomap_mapping_mission")
  {
    // PX4's ROS 2 topics use the sensor-data QoS profile (best effort).
    const rmw_qos_profile_t sensor_data_profile = rmw_qos_profile_sensor_data;
    const auto px4_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        sensor_data_profile.history, sensor_data_profile.depth),
      sensor_data_profile);

    offboard_control_mode_publisher_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos);
    trajectory_setpoint_publisher_ = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos);
    vehicle_command_publisher_ = create_publisher<VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos);

    manual_setpoint_subscription_ = create_subscription<TrajectorySetpoint>(
      "/octomap_mapping_mission/manual_setpoint",
      rclcpp::QoS(10),
      [this](const TrajectorySetpoint::SharedPtr message) {
        manual_setpoint_callback(message);
      });

    timer_ = create_wall_timer(
      kTimerPeriod,
      [this]() {
        timer_callback();
      });

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

  // PX4 positions use the NED convention:
  //   +X is north, +Y is east, and +Z is down.
  // Therefore, Z = -2.5 commands an altitude of 2.5 metres.
  //
  // These positions trace a 4 m square in the Gazebo world. Confirm that the
  // square is collision-free in the active world before running this node.
  static constexpr std::array<Waypoint, 4> kWaypoints{{
    {{{0.0F, 0.0F, -2.5F}}, 0.0F},
    {{{4.0F, 0.0F, -2.5F}}, 0.0F},
    {{{4.0F, 4.0F, -2.5F}}, 1.5707963F},
    {{{0.0F, 4.0F, -2.5F}}, 3.1415927F},
  }};

  // The timer runs at 10 Hz. Fifty ticks therefore command each waypoint for
  // five seconds. The fourth waypoint begins at 15 seconds, at which point the
  // node also starts accepting manually published setpoints.
  static constexpr std::chrono::milliseconds kTimerPeriod{100};
  static constexpr uint64_t kOffboardRequestTick = 10;
  static constexpr uint64_t kTicksPerWaypoint = 50;
  static constexpr uint64_t kManualControlStartTick =
    kTicksPerWaypoint * (kWaypoints.size() - 1);

  // Called by timer_ every 100 ms.
  void timer_callback()
  {
    // PX4 expects both messages to be streamed continuously while it is in
    // Offboard mode.
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    // PX4 requires setpoints to be streamed before it accepts an Offboard mode
    // request. This node deliberately does not arm the vehicle; arm it in QGC.
    if (offboard_setpoint_counter_ == kOffboardRequestTick) {
      publish_vehicle_command(
        VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      RCLCPP_INFO(get_logger(), "Requested PX4 Offboard mode");
    }

    ++offboard_setpoint_counter_;
  }

  // Called whenever a message arrives on the manual-setpoint ROS topic.
  void manual_setpoint_callback(const TrajectorySetpoint::SharedPtr message)
  {
    if (!scheduled_sequence_complete()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring manual setpoint: the four-waypoint sequence is still active");
      return;
    }

    const bool position_is_finite = std::all_of(
      message->position.begin(),
      message->position.end(),
      [](float value) {
        return std::isfinite(value);
      });

    if (!position_is_finite || !std::isfinite(message->yaw)) {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring manual setpoint: position and yaw must all be finite");
      return;
    }

    manual_waypoint_.position_ned = message->position;
    manual_waypoint_.yaw_ned = message->yaw;
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
    message.timestamp = current_time_in_microseconds();

    offboard_control_mode_publisher_->publish(message);
  }

  void publish_trajectory_setpoint()
  {
    const std::size_t waypoint_index = current_waypoint_index();
    const bool use_manual_waypoint =
      scheduled_sequence_complete() && manual_setpoint_received_;

    const Waypoint & waypoint = use_manual_waypoint ?
      manual_waypoint_ : kWaypoints[waypoint_index];

    if (!use_manual_waypoint && waypoint_index != last_logged_waypoint_index_) {
      RCLCPP_INFO(
        get_logger(),
        "Commanding waypoint %zu/%zu: NED=(%.1f, %.1f, %.1f), yaw=%.2f rad",
        waypoint_index + 1,
        kWaypoints.size(),
        waypoint.position_ned[0],
        waypoint.position_ned[1],
        waypoint.position_ned[2],
        waypoint.yaw_ned);
      last_logged_waypoint_index_ = waypoint_index;
    }

    TrajectorySetpoint message{};
    message.position = waypoint.position_ned;
    message.yaw = waypoint.yaw_ned;
    message.timestamp = current_time_in_microseconds();

    trajectory_setpoint_publisher_->publish(message);
  }

  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0F,
    float param2 = 0.0F)
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
    message.timestamp = current_time_in_microseconds();

    vehicle_command_publisher_->publish(message);
  }

  std::size_t current_waypoint_index() const
  {
    const std::size_t calculated_index =
      offboard_setpoint_counter_ / kTicksPerWaypoint;
    return std::min(calculated_index, kWaypoints.size() - 1);
  }

  bool scheduled_sequence_complete() const
  {
    return offboard_setpoint_counter_ >= kManualControlStartTick;
  }

  uint64_t current_time_in_microseconds()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  // ROS publishers, subscription, and timer.
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr manual_setpoint_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Mission state.
  uint64_t offboard_setpoint_counter_{0};
  std::size_t last_logged_waypoint_index_{
    std::numeric_limits<std::size_t>::max()};
  Waypoint manual_waypoint_{kWaypoints.back()};
  bool manual_setpoint_received_{false};
};

int main(int argc, char ** argv)
{
  std::cout << "Starting deterministic OctoMap mapping mission..." << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapMappingMission>());
  rclcpp::shutdown();
  return 0;
}

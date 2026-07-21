#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>

class GazeboPoseTfPublisher final : public rclcpp::Node
{
public:
  GazeboPoseTfPublisher()
  : Node("gazebo_pose_tf_publisher"),
    pose_topic_(declare_parameter<std::string>(
        "pose_topic", "/world/default/dynamic_pose/info")),
    model_name_(declare_parameter<std::string>("model_name", "x500_depth_0")),
    base_link_name_(declare_parameter<std::string>("base_link_name", "base_link")),
    world_frame_(declare_parameter<std::string>("world_frame", "world")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    const bool subscribed = gz_node_.Subscribe(
      pose_topic_, &GazeboPoseTfPublisher::pose_callback, this);

    if (!subscribed) {
      throw std::runtime_error("Failed to subscribe to Gazebo topic " + pose_topic_);
    }

    RCLCPP_INFO(
      get_logger(), "Publishing dynamic TF %s -> %s from %s",
      world_frame_.c_str(), base_frame_.c_str(), pose_topic_.c_str());
  }

private:
  static tf2::Transform to_tf(const gz::msgs::Pose & pose)
  {
    tf2::Quaternion rotation(
      pose.orientation().x(), pose.orientation().y(),
      pose.orientation().z(), pose.orientation().w());
    rotation.normalize();

    return tf2::Transform(
      rotation,
      tf2::Vector3(
        pose.position().x(), pose.position().y(), pose.position().z()));
  }

  void pose_callback(const gz::msgs::Pose_V & msg)
  {
    const gz::msgs::Pose * model_pose = nullptr;
    const gz::msgs::Pose * base_link_pose = nullptr;

    for (const auto & pose : msg.pose()) {
      if (pose.name() == model_name_) {
        model_pose = &pose;
      } else if (pose.name() == base_link_name_) {
        base_link_pose = &pose;
      }
    }

    if (model_pose == nullptr || base_link_pose == nullptr) {
      if (!missing_pose_warning_emitted_) {
        RCLCPP_WARN(
          get_logger(),
          "Waiting for Gazebo poses named '%s' and '%s' on %s",
          model_name_.c_str(), base_link_name_.c_str(), pose_topic_.c_str());
        missing_pose_warning_emitted_ = true;
      }
      return;
    }
    missing_pose_warning_emitted_ = false;

    // SceneBroadcaster poses are relative to each entity's parent:
    // world -> model composed with model -> base_link gives world -> base_link.
    const tf2::Transform world_to_base = to_tf(*model_pose) * to_tf(*base_link_pose);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp.sec = static_cast<int32_t>(msg.header().stamp().sec());
    transform.header.stamp.nanosec = static_cast<uint32_t>(msg.header().stamp().nsec());
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = base_frame_;

    const auto & translation = world_to_base.getOrigin();
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();

    const auto & rotation = world_to_base.getRotation();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform);
  }

  std::string pose_topic_;
  std::string model_name_;
  std::string base_link_name_;
  std::string world_frame_;
  std::string base_frame_;
  bool missing_pose_warning_emitted_{false};
  gz::transport::Node gz_node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPoseTfPublisher>());
  rclcpp::shutdown();
  return 0;
}

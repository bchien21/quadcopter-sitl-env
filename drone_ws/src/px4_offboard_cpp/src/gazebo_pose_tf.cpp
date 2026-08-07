#include <cstdint>
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

class GazeboPoseTfPublisher : public rclcpp::Node
{
public:
  GazeboPoseTfPublisher()
  : Node("gazebo_pose_tf_publisher")
  {
    // These defaults match the PX4 x500_depth Gazebo simulation.
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/world/default/dynamic_pose/info");
    model_name_ = declare_parameter<std::string>(
      "model_name", "x500_depth_0");
    base_link_name_ = declare_parameter<std::string>(
      "base_link_name", "base_link");
    world_frame_ = declare_parameter<std::string>(
      "world_frame", "world");
    base_frame_ = declare_parameter<std::string>(
      "base_frame", "base_link");

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Gazebo Transport is separate from ROS 2, so this is a Gazebo
    // subscription rather than an rclcpp::Subscription.
    const bool subscription_created = gz_node_.Subscribe(
      pose_topic_, &GazeboPoseTfPublisher::pose_callback, this);

    if (!subscription_created) {
      throw std::runtime_error(
              "Failed to subscribe to Gazebo topic " + pose_topic_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Publishing dynamic TF %s -> %s from %s",
      world_frame_.c_str(),
      base_frame_.c_str(),
      pose_topic_.c_str());
  }

private:
  // Called by Gazebo Transport whenever a new Pose_V message is received.
  void pose_callback(const gz::msgs::Pose_V & message)
  {
    const gz::msgs::Pose * model_pose = nullptr;
    const gz::msgs::Pose * base_link_pose = nullptr;

    // A Pose_V message contains poses for many Gazebo entities. Find the two
    // entities needed to calculate world -> base_link.
    for (const auto & pose : message.pose()) {
      if (pose.name() == model_name_) {
        model_pose = &pose;
      } else if (pose.name() == base_link_name_) {
        base_link_pose = &pose;
      }
    }

    if (model_pose == nullptr || base_link_pose == nullptr) {
      warn_about_missing_poses_once();
      return;
    }
    missing_pose_warning_emitted_ = false;

    // Gazebo's SceneBroadcaster reports each entity relative to its parent.
    // Compose world -> model and model -> base_link to obtain the transform
    // that ROS needs: world -> base_link.
    const tf2::Transform world_to_model = gazebo_pose_to_tf(*model_pose);
    const tf2::Transform model_to_base_link = gazebo_pose_to_tf(*base_link_pose);
    const tf2::Transform world_to_base_link =
      world_to_model * model_to_base_link;

    geometry_msgs::msg::TransformStamped transform_message{};
    transform_message.header.stamp.sec =
      static_cast<int32_t>(message.header().stamp().sec());
    transform_message.header.stamp.nanosec =
      static_cast<uint32_t>(message.header().stamp().nsec());
    transform_message.header.frame_id = world_frame_;
    transform_message.child_frame_id = base_frame_;

    const tf2::Vector3 & translation = world_to_base_link.getOrigin();
    transform_message.transform.translation.x = translation.x();
    transform_message.transform.translation.y = translation.y();
    transform_message.transform.translation.z = translation.z();

    const tf2::Quaternion & rotation = world_to_base_link.getRotation();
    transform_message.transform.rotation.x = rotation.x();
    transform_message.transform.rotation.y = rotation.y();
    transform_message.transform.rotation.z = rotation.z();
    transform_message.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_message);
  }

  tf2::Transform gazebo_pose_to_tf(const gz::msgs::Pose & pose) const
  {
    tf2::Quaternion rotation(
      pose.orientation().x(),
      pose.orientation().y(),
      pose.orientation().z(),
      pose.orientation().w());
    rotation.normalize();

    const tf2::Vector3 translation(
      pose.position().x(),
      pose.position().y(),
      pose.position().z());

    return tf2::Transform(rotation, translation);
  }

  void warn_about_missing_poses_once()
  {
    if (missing_pose_warning_emitted_) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Waiting for Gazebo poses named '%s' and '%s' on %s",
      model_name_.c_str(),
      base_link_name_.c_str(),
      pose_topic_.c_str());
    missing_pose_warning_emitted_ = true;
  }

  // Parameters.
  std::string pose_topic_;
  std::string model_name_;
  std::string base_link_name_;
  std::string world_frame_;
  std::string base_frame_;

  // Gazebo subscription and ROS TF publisher.
  gz::transport::Node gz_node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool missing_pose_warning_emitted_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPoseTfPublisher>());
  rclcpp::shutdown();
  return 0;
}

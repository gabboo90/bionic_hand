#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "Leap.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace Leap;
using namespace std::chrono_literals;

class LeapListener : public Listener {
public:
  LeapListener(
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
    : pose_publisher_(pose_pub),
      marker_publisher_(marker_pub),
      last_publish_time_(rclcpp::Clock().now()) {}

  void onFrame(const Controller& controller) override {
    rclcpp::Time now = rclcpp::Clock().now();
    if ((now - last_publish_time_).seconds() < 0.05) return;  // max 20 Hz
    last_publish_time_ = now;

    const Frame frame = controller.frame();
    if (frame.hands().isEmpty()) return;

    for (const Hand& hand : frame.hands()) {
      if (!hand.isRight()) continue;

      // === Handpose senden ===
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = now;
      pose_msg.header.frame_id = "leap_frame";

      pose_msg.pose.position.x = hand.palmPosition().x / 1000.0;
      pose_msg.pose.position.y = hand.palmPosition().y / 1000.0;
      pose_msg.pose.position.z = hand.palmPosition().z / 1000.0;

      float pitch = hand.direction().pitch();
      float roll = hand.palmNormal().roll();
      float yaw = hand.direction().yaw();

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.w = q.w();

      pose_publisher_->publish(pose_msg);

      // === MarkerArray erzeugen ===
      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;

      for (int i = 0; i <= 3; ++i) {  // Daumen bis Ringfinger
        const Finger& finger = hand.fingers()[i];

        for (int bone_type = 0; bone_type < 4; ++bone_type) {
          const Bone& bone = finger.bone(static_cast<Bone::Type>(bone_type));

          visualization_msgs::msg::Marker arrow;
          arrow.header.stamp = now;
          arrow.header.frame_id = "leap_frame";
          arrow.ns = "finger_arrow_" + std::to_string(i);
          arrow.id = marker_id++;
          arrow.type = visualization_msgs::msg::Marker::ARROW;
          arrow.action = visualization_msgs::msg::Marker::ADD;

          arrow.scale.x = 0.005;  // Schaftdurchmesser
          arrow.scale.y = 0.01;   // Kopfbreite
          arrow.scale.z = 0.02;   // Kopflänge

          arrow.color.r = 0.9;
          arrow.color.g = 0.3 + 0.2 * i;
          arrow.color.b = 1.0 - 0.2 * i;
          arrow.color.a = 1.0;

          geometry_msgs::msg::Point start, end;
          start.x = bone.prevJoint().x / 1000.0;
          start.y = bone.prevJoint().y / 1000.0;
          start.z = bone.prevJoint().z / 1000.0;

          end.x = bone.nextJoint().x / 1000.0;
          end.y = bone.nextJoint().y / 1000.0;
          end.z = bone.nextJoint().z / 1000.0;

          arrow.points.push_back(start);
          arrow.points.push_back(end);

          marker_array.markers.push_back(arrow);
        }
      }

      marker_publisher_->publish(marker_array);
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Time last_publish_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("leap_example");

  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/right_hand_pose_stamped", 10);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/leap_markers", 10);

  RCLCPP_INFO(node->get_logger(), "Leap Motion node started.");

  LeapListener listener(pose_pub, marker_pub);
  Controller controller;
  controller.addListener(listener);

  rclcpp::spin(node);

  controller.removeListener(listener);
  rclcpp::shutdown();
  return 0;
}

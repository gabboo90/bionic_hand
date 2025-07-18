#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "Leap.h"
#include "leap_node/msg/hand_data.hpp"
#include "leap_node/msg/finger_data.hpp"
#include "leap_node/msg/bone_data.hpp"

using namespace Leap;
using namespace std::chrono_literals;

class MCPPublisher : public rclcpp::Node {
public:
  MCPPublisher() : Node("mcp_publisher") {
    pub_ = this->create_publisher<leap_node::msg::HandData>("/mcp_angles", 10);
    controller_.addListener(listener_);
    timer_ = this->create_wall_timer(50ms, std::bind(&MCPPublisher::publish, this));
  }

  ~MCPPublisher() {
    controller_.removeListener(listener_);
  }

private:
  leap_node::msg::HandData hand_data_;

  class LeapListener : public Listener {
  public:
    LeapListener(leap_node::msg::HandData &data) : data_(data) {}

    void onFrame(const Controller &controller) override {
      const Frame frame = controller.frame();
      if (frame.hands().isEmpty()) return;

      for (const Hand &hand : frame.hands()) {
        if (!hand.isRight()) continue;

        data_.timestamp = rclcpp::Clock().now();
        data_.hand_id = hand.id();
        data_.is_right = true;

        data_.palm_position.x = hand.palmPosition().x / 1000.0;
        data_.palm_position.y = hand.palmPosition().y / 1000.0;
        data_.palm_position.z = hand.palmPosition().z / 1000.0;

        data_.normal.x = hand.palmNormal().x;
        data_.normal.y = hand.palmNormal().y;
        data_.normal.z = hand.palmNormal().z;

        data_.direction.x = hand.direction().x;
        data_.direction.y = hand.direction().y;
        data_.direction.z = hand.direction().z;

        for (int i = 0; i <= 2; ++i) {
          const Finger &finger = hand.fingers()[i];
          leap_node::msg::FingerData finger_msg;

          finger_msg.timestamp = data_.timestamp;
          finger_msg.finger_type = i;
          finger_msg.length = finger.length();
          finger_msg.width = finger.width();

          const Bone &proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
          Vector dir = proximal.direction();

          finger_msg.mcp_pitch_deg = dir.pitch() * 180.0 / M_PI;
          finger_msg.mcp_yaw_deg = dir.yaw() * 180.0 / M_PI;

          if (i == 0) data_.thumb = finger_msg;
          if (i == 1) data_.index = finger_msg;
          if (i == 2) data_.middle = finger_msg;
        }
      }
    }

  private:
    leap_node::msg::HandData &data_;
  };

  void publish() {
    pub_->publish(hand_data_);
  }

  Controller controller_;
  LeapListener listener_{hand_data_};
  rclcpp::Publisher<leap_node::msg::HandData>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCPPublisher>());
  rclcpp::shutdown();
  return 0;
}

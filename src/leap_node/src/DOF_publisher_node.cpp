#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "Leap.h"

using namespace Leap;
using namespace std::chrono_literals;

class MiddleFingerJointPublisher : public rclcpp::Node {
public:
  MiddleFingerJointPublisher() : Node("middle_finger_joint_publisher") {
    // Joint State Publisher für RViz2
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    controller_.addListener(listener_);
    timer_ = this->create_wall_timer(33ms, std::bind(&MiddleFingerJointPublisher::publish_joints, this));
    
    RCLCPP_INFO(this->get_logger(), "Middle finger joint publisher started (30Hz)");
  }

  ~MiddleFingerJointPublisher() {
    controller_.removeListener(listener_);
  }

private:
  // Joint States für Mittelfinger
  double middle_mcp_adduction_ = 0.0;
  double middle_mcp_flexion_ = 0.0;
  double middle_pip_ = 0.0;
  double middle_dip_ = 0.0;

  // Zeitstempel der letzten Hand-Erkennung
  std::chrono::steady_clock::time_point last_hand_time_ = std::chrono::steady_clock::now();
  
  class LeapListener : public Listener {
  public:
    LeapListener(MiddleFingerJointPublisher* parent) : parent_(parent) {}

    void onFrame(const Controller &controller) override {
      const Frame frame = controller.frame();
      if (frame.hands().isEmpty()) return;

      for (const Hand &hand : frame.hands()) {
        if (!hand.isRight()) continue; // Nur rechte Hand

        // Mittelfinger (Index 2 in Leap Motion)
        if (hand.fingers().count() > 2) {
          const Finger &middle_finger = hand.fingers()[2];
          
          // MCP Joint (Metacarpophalangeal) - Grundgelenk
          const Bone &metacarpal = middle_finger.bone(Bone::Type::TYPE_METACARPAL);
          const Bone &proximal = middle_finger.bone(Bone::Type::TYPE_PROXIMAL);
          
          // Berechne Winkel basierend auf Finger-Richtung
          Vector mcp_dir = proximal.direction();
          Vector pip_dir = middle_finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
          Vector dip_dir = middle_finger.bone(Bone::Type::TYPE_DISTAL).direction();
          
          // Konvertiere zu Joint-Winkeln (angepasst für URDF)
          parent_->middle_mcp_adduction_ = 0.0; // Seitliche Bewegung - erstmal 0
          
          // Robuste Winkel-Berechnung mit atan2 - verhindert Sprünge
          double raw_pitch = mcp_dir.pitch();
          double continuous_pitch;
          
          // Kontinuierlicher Winkel ohne Sprünge bei ±π
          if (raw_pitch > 0) {
              continuous_pitch = raw_pitch;
          } else {
              continuous_pitch = raw_pitch + 2.0 * M_PI; // Verschiebe negative Werte [0, 2π]
          }
          // double flexion_offset = -3.14; // Beispiel: 0.5 Radiant Offset
          // Flexion als Abstand vom neutralen Punkt (π = flache Hand)
          double flexion = std::min(continuous_pitch, 2.0 * M_PI - continuous_pitch) + flexion_offset;
          
          RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "raw_pitch: %f, continuous: %f, flexion: %f", 
                      raw_pitch, continuous_pitch, flexion);
          
          parent_->middle_mcp_flexion_ = std::max(0.0, std::min(1.5, flexion));
          parent_->middle_pip_ = std::abs(mcp_dir.angleTo(pip_dir)); // PIP Winkel  
          parent_->middle_dip_ = std::abs(pip_dir.angleTo(dip_dir)); // DIP Winkel
          
          // Zeitstempel aktualisieren
          parent_->last_hand_time_ = std::chrono::steady_clock::now();
        }
      }
    }

  private:
    MiddleFingerJointPublisher* parent_;
  };

  void publish_joints() {
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->now();
    
    // Mittelfinger Joints (entsprechend deiner URDF)
    joint_msg.name = {
      "middle_mcp_adduction",
      "middle_mcp_flexion", 
      "middle_pip",
      "middle_dip"
    };
    
    joint_msg.position = {
      middle_mcp_adduction_,
      middle_mcp_flexion_,
      middle_pip_,
      middle_dip_
    };
    
    // Velocities (optional)
    joint_msg.velocity = {0.0, 0.0, 0.0, 0.0};
    
    joint_pub_->publish(joint_msg);
  }

  Controller controller_;
  LeapListener listener_{this};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiddleFingerJointPublisher>());
  rclcpp::shutdown();
  return 0;
}
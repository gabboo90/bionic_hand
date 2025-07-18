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
  // === Kalibrierung: Offsets & Multiplikatoren für alle Finger und DOFs ===
  // Mittelfinger (middle)
  double middle_mcp_flexion_offset_ = -3.14;
  double middle_mcp_flexion_multiplier_ = -1.0;
  double middle_mcp_adduction_offset_ = -3.14;
  double middle_mcp_adduction_multiplier_ = 1.0;
  double middle_pip_offset_ = 0.0;
  double middle_pip_multiplier_ = 1.0;
  double middle_dip_offset_ = 0.0;
  double middle_dip_multiplier_ = 1.0;

  // Zeigefinger (index)
  double index_mcp_flexion_offset_ = -3.14;
  double index_mcp_flexion_multiplier_ = -1.0;
  double index_mcp_adduction_offset_ = -3.14;
  double index_mcp_adduction_multiplier_ = 1.0;
  double index_pip_offset_ = 0.0;
  double index_pip_multiplier_ = 1.0;
  double index_dip_offset_ = 0.0;
  double index_dip_multiplier_ = 1.0;

  // Ringfinger (ring)
  double ring_mcp_flexion_offset_ = -3.14;
  double ring_mcp_flexion_multiplier_ = -1.0;
  double ring_mcp_adduction_offset_ = -3.14;
  double ring_mcp_adduction_multiplier_ = 1.0;
  double ring_pip_offset_ = 0.0;
  double ring_pip_multiplier_ = 1.0;
  double ring_dip_offset_ = 0.0;
  double ring_dip_multiplier_ = 1.0;

  // Daumen (thumb)
  double thumb_opposition_offset_ = 0.0;
  double thumb_opposition_multiplier_ = -3.0;
  double thumb_rotation_offset_ = -M_PI/2;
  double thumb_rotation_multiplier_ = -1.0;
  double thumb_mcp_flexion_offset_ = -3.14;
  double thumb_mcp_flexion_multiplier_ = -1.0;
  double thumb_pip_offset_ = 0.0;
  double thumb_pip_multiplier_ = 1.0;
  double thumb_dip_offset_ = 0.0;
  double thumb_dip_multiplier_ = 1.0;

  // === Joint States ===
  // Mittelfinger
  double middle_mcp_adduction_ = 0.0;
  double middle_mcp_flexion_ = 0.0;
  double middle_pip_ = 0.0;
  double middle_dip_ = 0.0;
  // Zeigefinger
  double index_mcp_adduction_ = 0.0;
  double index_mcp_flexion_ = 0.0;
  double index_pip_ = 0.0;
  double index_dip_ = 0.0;
  // Ringfinger
  double ring_mcp_adduction_ = 0.0;
  double ring_mcp_flexion_ = 0.0;
  double ring_pip_ = 0.0;
  double ring_dip_ = 0.0;
  // Daumen
  double thumb_opposition_ = 0.0;
  double thumb_rotation_ = 0.0;
  double thumb_mcp_flexion_ = 0.0;
  double thumb_pip_ = 0.0;
  double thumb_dip_ = 0.0;

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

        // Iteriere über alle Finger und verarbeite sie nach Index (wie im sample_node)
        RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "=== Hand erkannt, Finger-Anzahl: %d ===", hand.fingers().count());
        
        for (int i = 0; i < hand.fingers().count(); ++i) {
          const Finger &finger = hand.fingers()[i];
          RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Finger Index %d - Typ: %d", i, static_cast<int>(finger.type()));
          
          // Minimaler Test: Setze alle Finger auf einen Testwert
          if (i == 1) { // Zeigefinger
            parent_->index_mcp_flexion_ = 0.5; // Testwert
            RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Index MCP Flexion auf 0.5 gesetzt");
          }
          if (i == 3) { // Ringfinger
            parent_->ring_mcp_flexion_ = 0.5; // Testwert
            RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Ring MCP Flexion auf 0.5 gesetzt");
          }
          
          switch (i) {
            case 0: { // Daumen
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "=== Daumen verarbeitet (Index 0) ===");
              // === Daumen ===
              const Bone &thumb_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector thumb_mcp_dir = thumb_proximal.direction();
              Vector thumb_pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              Vector thumb_dip_dir = finger.bone(Bone::Type::TYPE_DISTAL).direction();

              // Thumb Opposition (Daumen-Spreizung)
              double thumb_raw_yaw = thumb_mcp_dir.yaw();
              double thumb_cont_yaw = (thumb_raw_yaw > 0) ? thumb_raw_yaw : thumb_raw_yaw + 2.0 * M_PI;
              double thumb_opposition = (thumb_cont_yaw + parent_->thumb_opposition_offset_) * parent_->thumb_opposition_multiplier_;
              parent_->thumb_opposition_ = thumb_opposition;

              // Thumb Rotation
              double thumb_raw_roll = thumb_mcp_dir.roll();
              double thumb_cont_roll = (thumb_raw_roll > 0) ? thumb_raw_roll : thumb_raw_roll + 2.0 * M_PI;
              double thumb_rotation = (thumb_cont_roll + parent_->thumb_rotation_offset_) * parent_->thumb_rotation_multiplier_;
              parent_->thumb_rotation_ = thumb_rotation;

              // Thumb MCP Flexion
              double thumb_raw_pitch = thumb_mcp_dir.pitch();
              double thumb_cont_pitch = (thumb_raw_pitch > 0) ? thumb_raw_pitch : thumb_raw_pitch + 2.0 * M_PI;
              double thumb_flexion = (thumb_cont_pitch + parent_->thumb_mcp_flexion_offset_) * parent_->thumb_mcp_flexion_multiplier_;
              parent_->thumb_mcp_flexion_ = thumb_flexion;

              // Thumb PIP
              parent_->thumb_pip_ = std::abs(thumb_mcp_dir.angleTo(thumb_pip_dir)) * parent_->thumb_pip_multiplier_ + parent_->thumb_pip_offset_;
              // Thumb DIP
              parent_->thumb_dip_ = std::abs(thumb_pip_dir.angleTo(thumb_dip_dir)) * parent_->thumb_dip_multiplier_ + parent_->thumb_dip_offset_;
              break;
            }

            case 1: { // Zeigefinger
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "=== Zeigefinger verarbeitet (Index 1) ===");
              // === Zeigefinger ===
              const Bone &index_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector index_mcp_dir = index_proximal.direction();
              Vector index_pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              Vector index_dip_dir = finger.bone(Bone::Type::TYPE_DISTAL).direction();

              // MCP Adduktion
              double index_raw_yaw = index_mcp_dir.yaw();
              double index_cont_yaw = (index_raw_yaw > 0) ? index_raw_yaw : index_raw_yaw + 2.0 * M_PI;
              double index_adduction = (index_cont_yaw + parent_->index_mcp_adduction_offset_) * parent_->index_mcp_adduction_multiplier_;
              parent_->index_mcp_adduction_ = index_adduction;

              // MCP Flexion
              double index_raw_pitch = index_mcp_dir.pitch();
              double index_cont_pitch = (index_raw_pitch > 0) ? index_raw_pitch : index_raw_pitch + 2.0 * M_PI;
              double index_flexion = (index_cont_pitch + parent_->index_mcp_flexion_offset_) * parent_->index_mcp_flexion_multiplier_;
              parent_->index_mcp_flexion_ = index_flexion;
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Index MCP Flexion: raw_pitch: %f, continuous: %f, flexion: %f", 
                          index_raw_pitch, index_cont_pitch, index_flexion);

              // PIP
              parent_->index_pip_ = std::abs(index_mcp_dir.angleTo(index_pip_dir)) * parent_->index_pip_multiplier_ + parent_->index_pip_offset_;
              // DIP
              parent_->index_dip_ = std::abs(index_pip_dir.angleTo(index_dip_dir)) * parent_->index_dip_multiplier_ + parent_->index_dip_offset_;
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Index PIP: %f, DIP: %f", parent_->index_pip_, parent_->index_dip_);
              break;
            }

            case 2: { // Mittelfinger
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "=== Mittelfinger verarbeitet (Index 2) ===");
              // === Mittelfinger ===
              const Bone &middle_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector mcp_dir = middle_proximal.direction();
              Vector pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              Vector dip_dir = finger.bone(Bone::Type::TYPE_DISTAL).direction();

              // MCP Adduktion
              double raw_yaw = mcp_dir.yaw();
              double continuous_yaw = (raw_yaw > 0) ? raw_yaw : raw_yaw + 2.0 * M_PI;
              double adduction = (continuous_yaw + parent_->middle_mcp_adduction_offset_) * parent_->middle_mcp_adduction_multiplier_;
              parent_->middle_mcp_adduction_ = adduction;

              // MCP Flexion
              double raw_pitch = mcp_dir.pitch();
              double continuous_pitch = (raw_pitch > 0) ? raw_pitch : raw_pitch + 2.0 * M_PI;
              double flexion = (continuous_pitch + parent_->middle_mcp_flexion_offset_) * parent_->middle_mcp_flexion_multiplier_;
              parent_->middle_mcp_flexion_ = flexion;

              // PIP
              parent_->middle_pip_ = std::abs(mcp_dir.angleTo(pip_dir)) * parent_->middle_pip_multiplier_ + parent_->middle_pip_offset_;
              // DIP
              parent_->middle_dip_ = std::abs(pip_dir.angleTo(dip_dir)) * parent_->middle_dip_multiplier_ + parent_->middle_dip_offset_;
              break;
            }

            case 3: { // Ringfinger
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "=== Ringfinger verarbeitet (Index 3) ===");
              // === Ringfinger ===
              const Bone &ring_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector ring_mcp_dir = ring_proximal.direction();
              Vector ring_pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              Vector ring_dip_dir = finger.bone(Bone::Type::TYPE_DISTAL).direction();

              // MCP Adduktion
              double ring_raw_yaw = ring_mcp_dir.yaw();
              double ring_cont_yaw = (ring_raw_yaw > 0) ? ring_raw_yaw : ring_raw_yaw + 2.0 * M_PI;
              double ring_adduction = (ring_cont_yaw + parent_->ring_mcp_adduction_offset_) * parent_->ring_mcp_adduction_multiplier_;
              parent_->ring_mcp_adduction_ = ring_adduction;

              // MCP Flexion
              double ring_raw_pitch = ring_mcp_dir.pitch();
              double ring_cont_pitch = (ring_raw_pitch > 0) ? ring_raw_pitch : ring_raw_pitch + 2.0 * M_PI;
              double ring_flexion = (ring_cont_pitch + parent_->ring_mcp_flexion_offset_) * parent_->ring_mcp_flexion_multiplier_;
              parent_->ring_mcp_flexion_ = ring_flexion;
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Ring MCP Flexion: raw_pitch: %f, continuous: %f, flexion: %f", 
                          ring_raw_pitch, ring_cont_pitch, ring_flexion);

              // PIP
              parent_->ring_pip_ = std::abs(ring_mcp_dir.angleTo(ring_pip_dir)) * parent_->ring_pip_multiplier_ + parent_->ring_pip_offset_;
              // DIP
              parent_->ring_dip_ = std::abs(ring_pip_dir.angleTo(ring_dip_dir)) * parent_->ring_dip_multiplier_ + parent_->ring_dip_offset_;
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "Ring PIP: %f, DIP: %f", parent_->ring_pip_, parent_->ring_dip_);
              break;
            }

            default:
              RCLCPP_INFO(rclcpp::get_logger("LeapDebug"), "=== Finger Index %d ignoriert ===", i);
              break;
          }
        }

        // Zeitstempel aktualisieren
        parent_->last_hand_time_ = std::chrono::steady_clock::now();
      }
    }

  private:
    MiddleFingerJointPublisher* parent_;
  };

  void publish_joints() {
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->now();
    
    // Debug: Zeige die aktuellen Werte vor dem Publizieren
    RCLCPP_INFO(this->get_logger(), "=== Vor dem Publizieren ===");
    RCLCPP_INFO(this->get_logger(), "Index MCP Flexion: %f", index_mcp_flexion_);
    RCLCPP_INFO(this->get_logger(), "Ring MCP Flexion: %f", ring_mcp_flexion_);
    
    // Alle Finger Joints (entsprechend deiner URDF)
    joint_msg.name = {
      // Zeigefinger
      "index_mcp_adduction",
      "index_mcp_flexion",
      "index_pip",
      "index_dip",
      // Mittelfinger
      "middle_mcp_adduction",
      "middle_mcp_flexion", 
      "middle_pip",
      "middle_dip",
      // Ringfinger
      "ring_mcp_adduction",
      "ring_mcp_flexion",
      "ring_pip",
      "ring_dip",
      // Daumen
      "thumb_opposition",
      "thumb_rotation",
      "thumb_mcp_flexion",
      "thumb_pip",
      "thumb_dip"
    };
    
    joint_msg.position = {
      // Zeigefinger (echt)
      index_mcp_adduction_,
      index_mcp_flexion_,
      index_pip_,
      index_dip_,
      // Mittelfinger (echt)
      middle_mcp_adduction_,
      middle_mcp_flexion_,
      middle_pip_,
      middle_dip_,
      // Ringfinger (echt)
      ring_mcp_adduction_,
      ring_mcp_flexion_,
      ring_pip_,
      ring_dip_,
      // Daumen (echt)
      thumb_opposition_,
      thumb_rotation_,
      thumb_mcp_flexion_,
      thumb_pip_,
      thumb_dip_
    };
    
    // Debug: Zeige die publizierten Werte
    RCLCPP_INFO(this->get_logger(), "=== Publizierte Werte ===");
    RCLCPP_INFO(this->get_logger(), "Index [0-3]: %f, %f, %f, %f", 
                joint_msg.position[0], joint_msg.position[1], joint_msg.position[2], joint_msg.position[3]);
    RCLCPP_INFO(this->get_logger(), "Ring [8-11]: %f, %f, %f, %f", 
                joint_msg.position[8], joint_msg.position[9], joint_msg.position[10], joint_msg.position[11]);
    
    // Velocities (optional)
    joint_msg.velocity = std::vector<double>(joint_msg.name.size(), 0.0);
    
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
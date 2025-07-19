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
    timer_ = this->create_wall_timer(10, std::bind(&MiddleFingerJointPublisher::publish_joints, this));
    
    RCLCPP_INFO(this->get_logger(), "Middle finger joint publisher started (28Hz)");
  }

  ~MiddleFingerJointPublisher() {
    controller_.removeListener(listener_);
  }

private:
  // === Kalibrierung: Offsets & Multiplikatoren für alle Finger und DOFs ===
  // Mittelfinger (middle)
  double middle_mcp_flexion_offset_ = -3.14;
  double middle_mcp_flexion_multiplier_ = 1.0;
  double middle_mcp_adduction_offset_ = -3.14;
  double middle_mcp_adduction_multiplier_ = 1.0;
  double middle_pip_offset_ = 0.0;
  double middle_pip_multiplier_ = -1.0;
  double middle_dip_offset_ = 0.0;
  double middle_dip_multiplier_ = 1.0;

  // Zeigefinger (index)
  double index_mcp_flexion_offset_ = -3.14;
  double index_mcp_flexion_multiplier_ = 1.0;
  double index_mcp_adduction_offset_ = -3.14;
  double index_mcp_adduction_multiplier_ = 1.0;
  double index_pip_offset_ = 0.0;
  double index_pip_multiplier_ = -1.0;
  double index_dip_offset_ = 0.0;
  double index_dip_multiplier_ = 1.0;

  // Ringfinger (ring)
  double ring_mcp_flexion_offset_ = -3.0;
  double ring_mcp_flexion_multiplier_ = 1.0;
  double ring_mcp_adduction_offset_ = -3.14;
  double ring_mcp_adduction_multiplier_ = 1.0;
  double ring_pip_offset_ = 0.0;
  double ring_pip_multiplier_ = -1.0;
  double ring_dip_offset_ = 0.0;
  double ring_dip_multiplier_ = 1.0;

  // Daumen (thumb)
  double thumb_opposition_offset_ = -2.62;
  double thumb_opposition_multiplier_ = 2.5;
  double thumb_rotation_offset_ = - M_PI/2;
  double thumb_rotation_multiplier_ = -0.33;
  double thumb_mcp_flexion_offset_ = 0.0;
  double thumb_mcp_flexion_multiplier_ = 1.00;
  double thumb_pip_offset_ = 0.0;
  double thumb_pip_multiplier_ = -1.33;
  double thumb_dip_offset_ = 0.0;
  double thumb_dip_multiplier_ = 1.0;

  // Mimic-Multiplier für DIP-Gelenke
  double mimic_multiplier_ = 0.9;

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

        // HIER deklarieren!
        Matrix palm_basis = hand.basis();

        // Iteriere über alle Finger und verarbeite sie nach Index (wie im sample_node)
        
        for (int i = 0; i < hand.fingers().count(); ++i) {
          const Finger &finger = hand.fingers()[i];
          
          // Minimaler Test: Setze alle Finger auf einen Testwert
          if (i == 1) { // Zeigefinger
            parent_->index_mcp_flexion_ = 0.5; // Testwert
          }
          if (i == 3) { // Ringfinger
            parent_->ring_mcp_flexion_ = 0.5; // Testwert
          }
          
          switch (i) {
            case 0: { // Daumen
              // === Daumen ===
              const Bone &thumb_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector thumb_mcp_dir = thumb_proximal.direction();
              Vector thumb_pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              // Transformiere ins Palm-Frame
              Vector thumb_mcp_dir_in_palm = palm_basis.rigidInverse().transformDirection(thumb_mcp_dir);
              Vector thumb_pip_dir_in_palm = palm_basis.rigidInverse().transformDirection(thumb_pip_dir);
              // Opposition
              // Annahme: Opposition ist die Projektion auf die Y-Achse im Palm-Frame
              double thumb_opposition = parent_->thumb_opposition_multiplier_ * thumb_mcp_dir_in_palm.y;
              parent_->thumb_opposition_ = thumb_opposition;
              // Rotation
              double thumb_raw_roll = thumb_mcp_dir_in_palm.roll();
              double thumb_cont_roll = (thumb_raw_roll > 0) ? thumb_raw_roll : thumb_raw_roll + 2.0 * M_PI;
              double thumb_rotation = (thumb_cont_roll + parent_->thumb_rotation_offset_) * parent_->thumb_rotation_multiplier_;
              parent_->thumb_rotation_ = thumb_rotation;
              // MCP Flexion (zurück zur ursprünglichen Pitch-Berechnung)
              double thumb_raw_pitch = thumb_mcp_dir_in_palm.pitch();
              double thumb_cont_pitch = (thumb_raw_pitch > 0) ? thumb_raw_pitch : thumb_raw_pitch + 2.0 * M_PI;
              // Annahme: thumb_mcp_dir_in_palm = Oppositionsrichtung
              //          thumb_pip_dir_in_palm = Flexionsrichtung
              double thumb_flexion = parent_->thumb_mcp_flexion_multiplier_ * thumb_pip_dir_in_palm.x + parent_->thumb_mcp_flexion_offset_;
              parent_->thumb_mcp_flexion_ = thumb_flexion;
              // PIP (zurück zur ursprünglichen Winkelberechnung)
              Vector palm_normal = hand.palmNormal();
              Vector palm_normal_in_palm = palm_basis.rigidInverse().transformDirection(palm_normal);
              // Oppositionsrichtung als Referenz
              Vector opposition_axis = thumb_mcp_dir_in_palm.normalized();
              Vector flexion_axis = palm_normal_in_palm.cross(thumb_mcp_dir_in_palm).normalized();
              double thumb_pip = parent_->thumb_pip_multiplier_ * thumb_pip_dir_in_palm.dot(flexion_axis) + parent_->thumb_pip_offset_;
              parent_->thumb_pip_ = thumb_pip;
              // DIP (Mimic)
              parent_->thumb_dip_ = parent_->mimic_multiplier_ * parent_->thumb_pip_;
              break;
            }

            case 1: { // Zeigefinger
              // === Zeigefinger ===
              const Bone &index_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector index_mcp_dir = index_proximal.direction();
              Vector index_pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              // Transformiere ins Palm-Frame
              Vector index_mcp_dir_in_palm = palm_basis.rigidInverse().transformDirection(index_mcp_dir);
              Vector index_pip_dir_in_palm = palm_basis.rigidInverse().transformDirection(index_pip_dir);
              // MCP Adduktion
              double index_raw_yaw = index_mcp_dir_in_palm.yaw();
              double index_cont_yaw = (index_raw_yaw > 0) ? index_raw_yaw : index_raw_yaw + 2.0 * M_PI;
              double index_adduction = (index_cont_yaw + parent_->index_mcp_adduction_offset_) * parent_->index_mcp_adduction_multiplier_;
              parent_->index_mcp_adduction_ = index_adduction;
              // MCP Flexion
              double index_raw_pitch = index_mcp_dir_in_palm.pitch();
              double index_cont_pitch = (index_raw_pitch > 0) ? index_raw_pitch : index_raw_pitch + 2.0 * M_PI;
              double index_flexion = (index_cont_pitch + parent_->index_mcp_flexion_offset_) * parent_->index_mcp_flexion_multiplier_;
              parent_->index_mcp_flexion_ = index_flexion;
              // PIP
              parent_->index_pip_ = std::abs(index_mcp_dir_in_palm.angleTo(index_pip_dir_in_palm)) * parent_->index_pip_multiplier_ + parent_->index_pip_offset_;
              // DIP (Mimic)
              parent_->index_dip_ = parent_->mimic_multiplier_ * parent_->index_pip_;
              break;
            }

            case 2: { // Mittelfinger
              // === Mittelfinger ===
              const Bone &middle_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector mcp_dir = middle_proximal.direction();
              Vector pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              // Transformiere ins Palm-Frame
              Vector mcp_dir_in_palm = palm_basis.rigidInverse().transformDirection(mcp_dir);
              Vector pip_dir_in_palm = palm_basis.rigidInverse().transformDirection(pip_dir);
              // MCP Adduktion
              double raw_yaw = mcp_dir_in_palm.yaw();
              double continuous_yaw = (raw_yaw > 0) ? raw_yaw : raw_yaw + 2.0 * M_PI;
              double adduction = (continuous_yaw + parent_->middle_mcp_adduction_offset_) * parent_->middle_mcp_adduction_multiplier_;
              parent_->middle_mcp_adduction_ = adduction;
              // MCP Flexion
              double raw_pitch = mcp_dir_in_palm.pitch();
              double continuous_pitch = (raw_pitch > 0) ? raw_pitch : raw_pitch + 2.0 * M_PI;
              double flexion = (continuous_pitch + parent_->middle_mcp_flexion_offset_) * parent_->middle_mcp_flexion_multiplier_;
              parent_->middle_mcp_flexion_ = flexion;
              // PIP
              parent_->middle_pip_ = std::abs(mcp_dir_in_palm.angleTo(pip_dir_in_palm)) * parent_->middle_pip_multiplier_ + parent_->middle_pip_offset_;
              // DIP (Mimic)
              parent_->middle_dip_ = parent_->mimic_multiplier_ * parent_->middle_pip_;
              break;
            }

            case 3: { // Ringfinger
              // === Ringfinger ===
              const Bone &ring_proximal = finger.bone(Bone::Type::TYPE_PROXIMAL);
              Vector ring_mcp_dir = ring_proximal.direction();
              Vector ring_pip_dir = finger.bone(Bone::Type::TYPE_INTERMEDIATE).direction();
              // Transformiere ins Palm-Frame
              Vector ring_mcp_dir_in_palm = palm_basis.rigidInverse().transformDirection(ring_mcp_dir);
              Vector ring_pip_dir_in_palm = palm_basis.rigidInverse().transformDirection(ring_pip_dir);
              // MCP Adduktion
              double ring_raw_yaw = ring_mcp_dir_in_palm.yaw();
              double ring_cont_yaw = (ring_raw_yaw > 0) ? ring_raw_yaw : ring_raw_yaw + 2.0 * M_PI;
              double ring_adduction = (ring_cont_yaw + parent_->ring_mcp_adduction_offset_) * parent_->ring_mcp_adduction_multiplier_;
              parent_->ring_mcp_adduction_ = ring_adduction;
              // MCP Flexion
              double ring_raw_pitch = ring_mcp_dir_in_palm.pitch();
              double ring_cont_pitch = (ring_raw_pitch > 0) ? ring_raw_pitch : ring_raw_pitch + 2.0 * M_PI;
              double ring_flexion = (ring_cont_pitch + parent_->ring_mcp_flexion_offset_) * parent_->ring_mcp_flexion_multiplier_;
              parent_->ring_mcp_flexion_ = ring_flexion;
              // PIP
              parent_->ring_pip_ = std::abs(ring_mcp_dir_in_palm.angleTo(ring_pip_dir_in_palm)) * parent_->ring_pip_multiplier_ + parent_->ring_pip_offset_;
              // DIP (Mimic)
              parent_->ring_dip_ = parent_->mimic_multiplier_ * parent_->ring_pip_;
              break;
            }

            default:
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
    RCLCPP_INFO(this->get_logger(), "Middle [4-7]: %f, %f, %f, %f", 
                joint_msg.position[4], joint_msg.position[5], joint_msg.position[6], joint_msg.position[7]);
    RCLCPP_INFO(this->get_logger(), "Ring [8-11]: %f, %f, %f, %f", 
                joint_msg.position[8], joint_msg.position[9], joint_msg.position[10], joint_msg.position[11]);
    RCLCPP_INFO(this->get_logger(), "Thumb [12-16]: %f, %f, %f, %f, %f", 
                joint_msg.position[12], joint_msg.position[13], joint_msg.position[14], joint_msg.position[15], joint_msg.position[16]);
    
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
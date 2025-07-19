#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include <map>
#include <string>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// (Hier: Includes für DynamixelSDK und SCServo einfügen)

class ServoMapperNode : public rclcpp::Node {
public:
    ServoMapperNode() : Node("servo_mapper_node") {
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ServoMapperNode::joint_callback, this, std::placeholders::_1));
        
        // Publisher für Dynamixel SetPosition-Kommandos
        set_position_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
        
        // Timer für kontrollierte Update-Rate (28Hz)
        timer_ = this->create_wall_timer(10ms, std::bind(&ServoMapperNode::timer_callback, this));
        
        // Init Dynamixel, SCServo, Mapping-Tabellen etc.
        RCLCPP_INFO(this->get_logger(), "ServoMapperNode gestartet (28Hz)!");
    }

private:
    // Beispiel-Mapping: Passe die Servo-IDs an deine Hardware an!
    std::map<std::string, int> dynamixel_map = {
        {"thumb_opposition", 1},
        {"thumb_rotation", 2},
        {"index_mcp_adduction", 3},
        {"middle_mcp_adduction", 4},
        {"ring_mcp_adduction", 5}
    };
    std::map<std::string, int> scservo_map = {
        {"index_mcp_flexion", 1},
        {"index_pip", 2},
        {"middle_mcp_flexion", 3},
        {"middle_pip", 4},
        {"ring_mcp_flexion", 5},
        {"ring_pip", 6},
        {"thumb_mcp_flexion", 7},
        {"thumb_pip", 8}
    };

    // Aktuelle Joint-Werte
    std::map<std::string, double> current_joint_values_;
    
    // Letzte gesendete Positionen (für Deadband-Filter)
    std::map<std::string, int> last_sent_positions_;
    
    // Deadband-Schwelle (minimale Änderung für Update)
    const double DEADBAND_THRESHOLD = 0.02; // 0.02 rad ≈ 1.15°

    // Beispiel-Skalierung: Passe die Bereiche an deine Mechanik an!
    int rad_to_dynamixel(double rad) {
        // Annahme: -1.57 bis +1.57 rad → 0 bis 4095
        int pos = static_cast<int>( (rad + 1.57) * (4095.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 4095);
    }
    int rad_to_scservo(double rad) {
        // Annahme: -1.57 bis +1.57 rad → 0 bis 1023
        int pos = static_cast<int>( (rad + 1.57) * (1023.0 / (2 * 1.57)) );
        return std::clamp(pos, 0, 1023);
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Speichere die Joint-Werte, aber sende sie nicht sofort
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint = msg->name[i];
            double value = msg->position[i];
            current_joint_values_[joint] = value;
        }
    }
    
    void timer_callback() {
        // Verarbeite alle Joint-Werte mit kontrollierter Rate
        for (const auto& [joint, value] : current_joint_values_) {
            if (dynamixel_map.count(joint)) {
                int servo_id = dynamixel_map[joint];
                int new_pos = rad_to_dynamixel(value);
                
                // Deadband-Filter: Nur senden wenn sich die Position signifikant geändert hat
                int last_pos = last_sent_positions_[joint];
                if (std::abs(new_pos - last_pos) > (DEADBAND_THRESHOLD * 4095.0 / (2 * 1.57))) {
                    // Publiziere SetPosition-Kommando für Dynamixel
                    auto set_position_msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();
                    set_position_msg.id = servo_id;
                    set_position_msg.position = new_pos;
                    set_position_publisher_->publish(set_position_msg);
                    
                    last_sent_positions_[joint] = new_pos;
                    RCLCPP_INFO(this->get_logger(), "Dynamixel %s (ID %d): %d", joint.c_str(), servo_id, new_pos);
                }
            } else if (scservo_map.count(joint)) {
                int servo_id = scservo_map[joint];
                int new_pos = rad_to_scservo(value);
                
                // Deadband-Filter für SCServo
                int last_pos = last_sent_positions_[joint];
                if (std::abs(new_pos - last_pos) > (DEADBAND_THRESHOLD * 1023.0 / (2 * 1.57))) {
                    // TODO: Sende an SCServo-Servo servo_id, Position new_pos
                    last_sent_positions_[joint] = new_pos;
                    RCLCPP_INFO(this->get_logger(), "SCServo %s (ID %d): %d", joint.c_str(), servo_id, new_pos);
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // TODO: SDK-Objekte, Ports etc.
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoMapperNode>());
    rclcpp::shutdown();
    return 0;
} 